#include <awnn_lib.h>
#include "yolov5_post_process.h"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>

using Clock = std::chrono::steady_clock;

struct CameraState {
    std::mutex mutex;
    cv::Mat latest;
    std::atomic<bool> running{true};
    std::atomic<unsigned long long> captured{0};
    unsigned long long frame_id{0};
};

static double elapsed_ms(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

static void camera_loop(CameraState* state, int index, int width, int height, int fps, bool flip_horizontal) {
    cv::VideoCapture cap(index, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::fprintf(stderr, "无法打开摄像头 /dev/video%d\n", index);
        state->running = false;
        return;
    }
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, fps);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    std::fprintf(stderr, "摄像头实际配置: %.0fx%.0f, %.1f FPS\n",
        cap.get(cv::CAP_PROP_FRAME_WIDTH), cap.get(cv::CAP_PROP_FRAME_HEIGHT), cap.get(cv::CAP_PROP_FPS));

    cv::Mat frame, prepared;
    while (state->running) {
        if (!cap.read(frame) || frame.empty()) continue;
        if (flip_horizontal) cv::flip(frame, prepared, 1);
        else prepared = frame;
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            prepared.copyTo(state->latest);
            state->frame_id++;
        }
        state->captured++;
    }
    cap.release();
}

static void preprocess_direct(const cv::Mat& bgr, unsigned char* dst) {
    constexpr int W=640, H=640;
    float scale = std::min(W / static_cast<float>(bgr.cols), H / static_cast<float>(bgr.rows));
    int rw = std::max(1, static_cast<int>(std::round(bgr.cols * scale)));
    int rh = std::max(1, static_cast<int>(std::round(bgr.rows * scale)));
    int left = (W-rw)/2, top = (H-rh)/2;

    cv::Mat resized, canvas(H, W, CV_8UC3, cv::Scalar(114,114,114)), rgb;
    cv::resize(bgr, resized, cv::Size(rw,rh), 0, 0, cv::INTER_LINEAR);
    resized.copyTo(canvas(cv::Rect(left,top,rw,rh)));
    cv::cvtColor(canvas, rgb, cv::COLOR_BGR2RGB);

    const int plane=W*H;
    for (int y=0; y<H; ++y) {
        const cv::Vec3b* row=rgb.ptr<cv::Vec3b>(y);
        for (int x=0; x<W; ++x) {
            const cv::Vec3b& p=row[x];
            int pos=y*W+x;
            dst[pos]=p[0];
            dst[plane+pos]=p[1];
            dst[2*plane+pos]=p[2];
        }
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr,
            "用法: %s <model.nb> <video索引> [秒数] [采集宽] [采集高] [请求FPS] [水平翻转0/1]\n",
            argv[0]);
        return 1;
    }
    const char* model=argv[1];
    int video_index=std::atoi(argv[2]);
    int seconds=argc>3?std::atoi(argv[3]):20;
    int width=argc>4?std::atoi(argv[4]):640;
    int height=argc>5?std::atoi(argv[5]):480;
    int fps=argc>6?std::atoi(argv[6]):120;
    bool flip_horizontal=argc>7?std::atoi(argv[7])!=0:true;

    CameraState cam;
    std::thread camera_thread(camera_loop,&cam,video_index,width,height,fps,flip_horizontal);
    while (cam.running) {
        {
            std::lock_guard<std::mutex> lock(cam.mutex);
            if (!cam.latest.empty()) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!cam.running) { camera_thread.join(); return 2; }

    awnn_init();
    Awnn_Context_t* context=awnn_create(model);
    if (!context) {
        cam.running=false; camera_thread.join(); awnn_uninit();
        std::fprintf(stderr,"NPU模型加载失败\n"); return 3;
    }

    auto* input=(unsigned char*)std::malloc(3*640*640);
    void* input_buffers[]={input};
    double pre_ms_sum=0, npu_ms_sum=0, post_ms_sum=0, chain_ms_sum=0;
    unsigned long long processed=0, last_processed=0, last_captured=cam.captured.load();
    unsigned long long last_frame_id=0;
    auto start=Clock::now(), last_report=start, end=start+std::chrono::seconds(seconds);

    while (cam.running && Clock::now()<end) {
        cv::Mat frame;
        unsigned long long current_frame_id;
        {
            std::lock_guard<std::mutex> lock(cam.mutex);
            current_frame_id = cam.frame_id;
            if (current_frame_id == last_frame_id)
                continue;
            cam.latest.copyTo(frame);
        }
        if (frame.empty()) continue;
        last_frame_id = current_frame_id;
        auto chain_begin=Clock::now();
        auto pre_begin=chain_begin;
        preprocess_direct(frame,input);
        auto pre_end=Clock::now();
        awnn_set_input_buffers(context,input_buffers);
        auto npu_begin=Clock::now();
        awnn_run(context);
        auto npu_end=Clock::now();
        float** outputs=awnn_get_output_buffers(context);
        auto post_begin=Clock::now();
        int ball_count=0;
        float best_confidence=0.0f;
        float best_x=-1.0f, best_y=-1.0f, best_w=0.0f, best_h=0.0f;
        yolov5_post_process_mat(frame,outputs,&ball_count,&best_confidence,&best_x,&best_y,&best_w,&best_h);
        auto post_end=Clock::now();
        auto chain_end=Clock::now();
        processed++;
        pre_ms_sum+=elapsed_ms(pre_begin,pre_end);
        npu_ms_sum+=elapsed_ms(npu_begin,npu_end);
        post_ms_sum+=elapsed_ms(post_begin,post_end);
        chain_ms_sum+=elapsed_ms(chain_begin,chain_end);

        auto now=Clock::now();
        double period=std::chrono::duration<double>(now-last_report).count();
        if (period>=1.0) {
            auto cap_now=cam.captured.load();
            double camera_fps=(cap_now-last_captured)/period;
            double realtime_fps=(processed-last_processed)/period;
            std::printf("采集 %.1f FPS | 预处理 %.2f ms | 纯NPU %.2f ms (%.1f FPS) | 优化链路 %.2f ms (实时 %.1f FPS)\n",
                camera_fps, pre_ms_sum/processed, npu_ms_sum/processed,
                1000.0/(npu_ms_sum/processed), chain_ms_sum/processed, realtime_fps);
            std::printf("Post-process %.2f ms | balls %d | best %.3f | center (%.1f, %.1f) | box %.1fx%.1f\n", post_ms_sum/processed, ball_count, best_confidence, best_x, best_y, best_w, best_h);
            std::fflush(stdout);
            last_captured=cap_now; last_processed=processed; last_report=now;
        }
    }

    cam.running=false; camera_thread.join();
    awnn_destroy(context); awnn_uninit(); std::free(input);
    double total_s=std::chrono::duration<double>(Clock::now()-start).count();
    std::printf("\n=== 汇总 ===\n");
    std::printf("分辨率: %dx%d, 水平翻转: %s\n",width,height,flip_horizontal?"是":"否");
    std::printf("平均采集FPS: %.2f\n",cam.captured.load()/total_s);
    std::printf("平均预处理: %.2f ms\n",processed?pre_ms_sum/processed:0);
    std::printf("平均纯NPU: %.2f ms, %.2f FPS\n",processed?npu_ms_sum/processed:0,npu_ms_sum>0?processed*1000.0/npu_ms_sum:0);
    std::printf("Average post-process: %.2f ms\n",processed?post_ms_sum/processed:0);
    std::printf("平均优化链路: %.2f ms, %.2f FPS\n",processed?chain_ms_sum/processed:0,total_s>0?processed/total_s:0);
    std::printf("说明: 优化链路包含采集帧复制、内存预处理、输入拷贝、NPU推理和输出读取；包含内存解码和NMS；不含绘框和图片写盘。\n");
    return 0;
}

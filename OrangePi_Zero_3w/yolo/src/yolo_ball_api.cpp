#include "yolo_ball_api.h"
#include "yolov5_post_process.h"
#include <awnn_lib.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <mutex>

using Clock = std::chrono::steady_clock;

struct YoloBallHandle {
    Awnn_Context_t *context;
    unsigned char *input;
    void *input_buffers[1];
    std::mutex mutex;
};

static float elapsed_ms(Clock::time_point begin, Clock::time_point end)
{
    return static_cast<float>(
        std::chrono::duration<double, std::milli>(end - begin).count()
    );
}

static void preprocess_bgr(const cv::Mat& bgr, unsigned char *dst)
{
    const int net_w = 640;
    const int net_h = 640;

    float scale = std::min(
        net_w / static_cast<float>(bgr.cols),
        net_h / static_cast<float>(bgr.rows)
    );

    int resize_w = std::max(
        1,
        static_cast<int>(std::round(bgr.cols * scale))
    );
    int resize_h = std::max(
        1,
        static_cast<int>(std::round(bgr.rows * scale))
    );

    int left = (net_w - resize_w) / 2;
    int top = (net_h - resize_h) / 2;

    cv::Mat resized;
    cv::Mat canvas(
        net_h,
        net_w,
        CV_8UC3,
        cv::Scalar(114, 114, 114)
    );
    cv::Mat rgb;

    cv::resize(
        bgr,
        resized,
        cv::Size(resize_w, resize_h),
        0,
        0,
        cv::INTER_LINEAR
    );

    resized.copyTo(
        canvas(cv::Rect(left, top, resize_w, resize_h))
    );

    cv::cvtColor(canvas, rgb, cv::COLOR_BGR2RGB);

    const int plane = net_w * net_h;

    for (int y = 0; y < net_h; ++y)
    {
        const cv::Vec3b *row = rgb.ptr<cv::Vec3b>(y);

        for (int x = 0; x < net_w; ++x)
        {
            const cv::Vec3b& pixel = row[x];
            int pos = y * net_w + x;

            dst[pos] = pixel[0];
            dst[plane + pos] = pixel[1];
            dst[2 * plane + pos] = pixel[2];
        }
    }
}

extern "C" void *yolo_ball_create(const char *model_path)
{
    if (!model_path)
        return nullptr;

    YoloBallHandle *handle = new YoloBallHandle();
    handle->context = nullptr;
    handle->input = nullptr;
    handle->input_buffers[0] = nullptr;

    awnn_init();

    handle->context = awnn_create(model_path);
    if (!handle->context)
    {
        awnn_uninit();
        delete handle;
        return nullptr;
    }

    handle->input = static_cast<unsigned char *>(
        std::malloc(3 * 640 * 640)
    );

    if (!handle->input)
    {
        awnn_destroy(handle->context);
        awnn_uninit();
        delete handle;
        return nullptr;
    }

    handle->input_buffers[0] = handle->input;
    return handle;
}

extern "C" int yolo_ball_detect(
    void *opaque_handle,
    const unsigned char *bgr,
    int width,
    int height,
    int stride,
    YoloBallResult *result)
{
    if (!opaque_handle || !bgr || !result)
        return -1;

    if (width <= 0 || height <= 0 || stride < width * 3)
        return -2;

    YoloBallHandle *handle =
        static_cast<YoloBallHandle *>(opaque_handle);

    std::lock_guard<std::mutex> lock(handle->mutex);

    cv::Mat frame(
        height,
        width,
        CV_8UC3,
        const_cast<unsigned char *>(bgr),
        static_cast<size_t>(stride)
    );

    result->count = 0;
    result->confidence = 0.0f;
    result->cx = -1.0f;
    result->cy = -1.0f;
    result->width = 0.0f;
    result->height = 0.0f;
    result->pre_ms = 0.0f;
    result->npu_ms = 0.0f;
    result->post_ms = 0.0f;

    auto pre_begin = Clock::now();
    
preprocess_bgr(frame, handle->input);
auto pre_end = Clock::now();
 
awnn_set_input_buffers(
handle->context,
handle->input_buffers
);
 
auto npu_begin = Clock::now();
awnn_run(handle->context);
auto npu_end = Clock::now();
 
float **outputs =
awnn_get_output_buffers(handle->context);
 
if (!outputs)
return -3;
 
auto post_begin = Clock::now();
 
int rc = yolov5_post_process_mat(
frame,
outputs,
&result->count,
&result->confidence,
&result->cx,
&result->cy,
&result->width,
&result->height
);
 
auto post_end = Clock::now();
 
result->pre_ms = elapsed_ms(pre_begin, pre_end);
result->npu_ms = elapsed_ms(npu_begin, npu_end);
result->post_ms = elapsed_ms(post_begin, post_end);
 
return rc;
}
 
extern "C" void yolo_ball_destroy(void *opaque_handle)
{
if (!opaque_handle)
return;
 
YoloBallHandle *handle =
static_cast<YoloBallHandle *>(opaque_handle);
 
if (handle->context)
awnn_destroy(handle->context);
 
std::free(handle->input);
awnn_uninit();
delete handle;
}

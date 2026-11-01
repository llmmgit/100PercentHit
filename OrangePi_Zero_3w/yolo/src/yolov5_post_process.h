#ifndef __YOLOV5_POST_PROCESS_H__
#define __YOLOV5_POST_PROCESS_H__
#ifdef __cplusplus
extern "C" {
#endif

int yolov5_post_process(const char *imagepath, float **output);

#ifdef __cplusplus
}
#endif

#endif
 
#ifdef __cplusplus
#include <opencv2/core.hpp>
 
int yolov5_post_process_mat(
const cv::Mat& image,
float **output,
int *ball_count,
float *best_confidence,
float *best_x = nullptr,
float *best_y = nullptr,
float *best_w = nullptr,
float *best_h = nullptr
);
#endif

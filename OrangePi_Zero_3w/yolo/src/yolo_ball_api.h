#ifndef YOLO_BALL_API_H
#define YOLO_BALL_API_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct YoloBallResult {
    int count;
    float confidence;
    float cx;
    float cy;
    float width;
    float height;
    float pre_ms;
    float npu_ms;
    float post_ms;
} YoloBallResult;

void *yolo_ball_create(const char *model_path);

int yolo_ball_detect(
    void *handle,
    const unsigned char *bgr,
    int width,
    int height,
    int stride,
    YoloBallResult *result
);

void yolo_ball_destroy(void *handle);

#ifdef __cplusplus
}
#endif

#endif

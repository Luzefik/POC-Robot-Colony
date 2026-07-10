#pragma once

#include "esp_camera.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One detected LED marker, in image-centered coordinates:
 * x: left negative / right positive, y: down negative / up positive. */
typedef struct {
    float x;
    float y;
    int count; /* pixels in the blob */
} DetectedDot;

typedef struct {
    bool valid;
    DetectedDot dots[3]; /* ordered left, center, right */
    float spacing_px;    /* dots[2].x - dots[0].x; grows as the leader gets closer */
} BlobResult;

/* Detect the three-LED reference on a YUV422 (YUYV) frame and publish the
 * result to dots_detection_queue (single-slot, overwrite semantics). */
BlobResult process_image(camera_fb_t *fb);

#ifdef __cplusplus
}
#endif

#pragma once
#include <stdint.h>
#include <stdlib.h>
#include "esp_camera.h"

typedef struct {
    int x;
    int y;
} Point;

typedef struct {
    int x;
    int y;
    int width;
    int height;
} dot_rect_t;

typedef struct {
    int id;
    Point center;
    int size;
} Blob;

typedef struct {
    int count;
    dot_rect_t *rects;
} detection_result_t;

void detect_dots(camera_fb_t *fb);
void detect_dots_in_frame(const uint8_t *jpeg_data, size_t jpeg_len,
                          int orig_width, int orig_height);
detection_result_t* get_detection_result(void);
Point get_center_point(void);
Point* get_dots(void);
int* get_dots_sizes(void);


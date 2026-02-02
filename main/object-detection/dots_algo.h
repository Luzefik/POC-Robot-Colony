#pragma once
#include <stdint.h>
#include <stdlib.h>
#include "esp_camera.h"

typedef struct {
    int x;
    int y;
} Point;

typedef struct {
    int id;
    Point center;
    int size;
} Blob;

typedef struct {
    int count;
} detection_result_t;

typedef struct {
    Point data[3];
    int sizes[3];
    int count;
} detection_data_t;

void detect_dots(camera_fb_t *fb);
detection_result_t* get_detection_result(void);
Point get_center_point(void);
Point* get_dots(void);
int* get_dots_sizes(void);
detection_data_t get_detection_data(void);


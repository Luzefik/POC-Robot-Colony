#pragma once
#include <stdint.h>
#include <stdlib.h>
#include "esp_camera.h"

// Структури (якщо потрібні ззовні)
typedef struct {
    int x;
    int y;
} Point;

typedef struct {
    Point center;
    int size;
    int id;
} Blob;

/**
 * @param fb - frame buffer from ESP32
 */
void detect_dots(camera_fb_t *fb);

/**
 * @param jpeg_data -  JPEG data
 * @param jpeg_len - size of JPEG
 * @param orig_width - original width
 * @param orig_height - original height
 */
void detect_dots_in_frame(const uint8_t *jpeg_data, size_t jpeg_len,
                          int orig_width, int orig_height);
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "esp_log.h"

#include "esp_heap_caps.h"
#include "esp_camera.h"

#include "take_picture.h"

static const char *TAG = "dots_algo";

#define RED_V_THRESH 170
#define LUMA_THRESH 40
#define CHROMA_U_LOW 100

struct Point {
    float_t x;
    float_t y;
    int number_of_points;
};


void process_image(camera_fb_t * fb) {
    if (!fb) {
        ESP_LOGE(TAG, "No frame buffer provided");
        return;
    }

    uint32_t sum_x_top = 0;
    uint32_t sum_x_bottom = 0;
    uint32_t red_pixel_count_top = 0;
    uint32_t red_pixel_count_bottom = 0;
    uint16_t mid_line = fb->height / 2;

    for ( int i = 0; i < fb->len; i +=4) {
        uint8_t V = fb->buf[i+3];
        // uint8_t Y = fb->buf[i+2];

        if (V > RED_V_THRESH) {
            int index_point = i / 2;
            int y = index_point / fb->width;
            int x = (i / 2) % fb->width;

            if (y < mid_line) {
                sum_x_top += x;
                red_pixel_count_top++;
                continue;
            }
            else {
                sum_x_bottom += x;
                red_pixel_count_bottom++;
            }
        }
    }
    int center_top = (red_pixel_count_top > 10) ? (sum_x_top / red_pixel_count_top) : -1;
    int center_bot = (red_pixel_count_bottom > 10) ? (sum_x_bottom / red_pixel_count_bottom) : -1;


    if (center_top != -1 && center_bot != -1) {

        int slope = center_top - center_bot;

        ESP_LOGI("TEST", "Top X: %d | Bot X: %d | Slope: %d", center_top, center_bot, slope);

        if (abs(slope) < 10) {
             ESP_LOGI("STATUS", "LINE IS STRAIGHT (Пряма лінія)");
        } else if (slope > 10) {
             ESP_LOGI("STATUS", "LINE TURNS RIGHT (Поворот вправо / Ми дивимось вліво)");
        } else {
             ESP_LOGI("STATUS", "LINE TURNS LEFT (Поворот вліво / Ми дивимось вправо)");
        }
    } else {
        ESP_LOGI("TEST", "Line not fully visible");
    }
}

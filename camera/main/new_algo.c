
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "esp_heap_caps.h"
#include "esp_camera.h"

#include "take_picture.h"

static const char *TAG = "dots_algo";

#define RED_V_THRESH 160
#define LUMA_THRESH 40
#define CHROMA_U_LOW 100
#define MAX_BLOBS 10

struct Blob {
    float_t cord_x;
    float_t cord_y;
    int sum_x;
    int sum_y;
    int count;
};

void process_image(camera_fb_t * fb) {
    float central_of_img_x = fb->width/2.0;
    float central_of_img_y = fb->height/2.0;


    struct Blob blobs[MAX_BLOBS];
    int active_blobs = 0;

    for(int k=0; k<MAX_BLOBS; k++) blobs[k].count = 0;

    if (!fb) {
        ESP_LOGE(TAG, "No frame buffer provided");
        return;
    }

    for ( int i = 0; i < fb->len; i +=4) {

        uint8_t Y0 = fb->buf[i];
        uint8_t U  = fb->buf[i+1];
        uint8_t Y1 = fb->buf[i+2];
        uint8_t V  = fb->buf[i+3];
        // [Y0][U][Y1][V]

            for (int pass = 0; pass < 2; pass++) {
                if ((V > RED_V_THRESH || V == RED_V_THRESH )&& U < CHROMA_U_LOW && Y0 > LUMA_THRESH) {

                // red pixel detected at position i/4
                int pixel_index = (i / 4)* 2 + pass;
                int x = pixel_index % fb->width;
                int y = pixel_index / fb->width;

                // Check if it belongs to an existing blob

                int best_blob_idx = -1;
                int min_dist_sq = 999999;

                for (int k = 0; k < active_blobs; k++) {
                    int dx = x - blobs[k].cord_x;
                    int dy = y - blobs[k].cord_y;
                    int dist_sq = dx * dx + dy * dy;

                    if (dist_sq < 100) { // within 10 pixels
                        if (dist_sq < min_dist_sq) {
                            min_dist_sq = dist_sq;
                            best_blob_idx = k;
                        }
                    }
                }
                if (best_blob_idx != -1) {
                    // Add to existing blob
                    blobs[best_blob_idx].sum_x += x;
                    blobs[best_blob_idx].sum_y += y;
                    blobs[best_blob_idx].count += 1;
                    blobs[best_blob_idx].cord_x = (float_t)blobs[best_blob_idx].sum_x / blobs[best_blob_idx].count;
                    blobs[best_blob_idx].cord_y = (float_t)blobs[best_blob_idx].sum_y / blobs[best_blob_idx].count;
                } else if (active_blobs < MAX_BLOBS) {
                    // Create new blob
                    blobs[active_blobs].cord_x = x;
                    blobs[active_blobs].cord_y = y;
                    blobs[active_blobs].sum_x = x;
                    blobs[active_blobs].sum_y = y;
                    blobs[active_blobs].count = 1;
                    active_blobs++;
                }
            }
        }
    }
// sorting blobs by size (count)
for (int i = 0; i < active_blobs - 1; i++) {
    for (int j = i + 1; j < active_blobs; j++) {
        if (blobs[j].count > blobs[i].count) {
            struct Blob temp = blobs[i];
            blobs[i] = blobs[j];
            blobs[j] = temp;
        }
    }
}

//sorting by x coordinate top 3 blobs
for (int i = 0; i < 2; i++) {
    for (int j = i+1; j < 3; j++) {
        if (blobs[j].cord_x < blobs[i].cord_x) {
            struct Blob temp = blobs[i];
            blobs[i] = blobs[j];
            blobs[j] = temp;
        }
    }
}

    static int64_t last_log_time = 0;
    int64_t current_time = esp_timer_get_time() / 1000;


    if (current_time - last_log_time > 500) {
    last_log_time = current_time;
if (active_blobs >= 3) {
    ESP_LOGI(TAG, "LEFT DOT:   X=%.1f, Y=%.1f", blobs[0].cord_x, blobs[0].cord_y);
    ESP_LOGI(TAG, "CENTER DOT: X=%.1f, Y=%.1f", blobs[1].cord_x, blobs[1].cord_y);
    ESP_LOGI(TAG, "RIGHT DOT:  X=%.1f, Y=%.1f", blobs[2].cord_x, blobs[2].cord_y);

    float slope = blobs[2].cord_y - blobs[0].cord_y;
    ESP_LOGI(TAG, "Angle Slope: %.2f", slope);

} else {
    ESP_LOGW(TAG, "NOT ENOUGH DOTS (Need 3, found %d)", active_blobs);
}
}}
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "new_algo.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "freertos/idf_additions.h"
#include "esp_timer.h"
#include "take_picture.h"

static const char *TAG = "dots_algo";

extern QueueHandle_t dots_detection_queue;

#define RED_V_THRESH 150
// red pixel thresholds 150 - для яскаво червного
#define LUMA_THRESH 50
// luma threshold 40, але для темноти 50
#define CHROMA_U_LOW 100
#define MAX_BLOBS 5
#define MAX_BLIND_FRAMES 5

static int blind_frame_counter = 0;
static BlobResult last_good_result = {0};
static bool found_blobs = false;


/*
  Replaced process_image with a faster scanning/clustering implementation.
  Uses a small integer FastBlob accumulator to reduce float ops during scan,
  then converts to Blob (float) for pattern matching / queue send.
*/

typedef struct {
    long sum_x;
    long sum_y;
    int count;
    int id;
} FastBlob;

#define SCAN_STEP 2        // 1 = every pixel, 2 = skip every other (faster)
#define FAST_MIN_COUNT 5   // min pixels for a fast blob to be considered

BlobResult process_image(camera_fb_t * fb) {
    BlobResult result = {0};

    if (!fb) {
        ESP_LOGE(TAG, "No frame buffer provided");
        xQueueSend(dots_detection_queue, &result, portMAX_DELAY);
        return result;
    }

    int width = fb->width;
    int height = fb->height;
    uint8_t *buf = fb->buf;

    FastBlob fast_blobs[MAX_BLOBS];
    for (int k = 0; k < MAX_BLOBS; k++) {
        fast_blobs[k].sum_x = 0;
        fast_blobs[k].sum_y = 0;
        fast_blobs[k].count = 0;
        fast_blobs[k].id = k;
    }
    int active_fast = 0;


    for (int y = 0; y < height; y += SCAN_STEP) {
        int row_offset = y * width * 2; // YUV422: 2 bytes per pixel (4 bytes per 2 pixels)

        for (int x = 0; x < width; x += SCAN_STEP) {
            int i = row_offset + (x * 2);
            if (i + 3 >= fb->len) break;

            uint8_t Y = buf[i];       // Y for pixel x (when x is even this is correct for Y0)
            uint8_t U = buf[i + 1];
            uint8_t V = buf[i + 3];

            if (V >= RED_V_THRESH && U < CHROMA_U_LOW && Y > LUMA_THRESH) {
                // cluster into fast_blobs
                int best_idx = -1;
                long best_dist = 2500; // 50^2 default threshold

                for (int b = 0; b < active_fast; b++) {
                    int cx = fast_blobs[b].sum_x / fast_blobs[b].count;
                    int cy = fast_blobs[b].sum_y / fast_blobs[b].count;
                    int dx = x - cx;
                    int dy = y - cy;
                    if (abs(dx) > 50 || abs(dy) > 50) continue;
                    long d2 = dx*dx + dy*dy;
                    if (d2 < best_dist) {
                        best_dist = d2;
                        best_idx = b;
                    }
                }

                if (best_idx != -1) {
                    fast_blobs[best_idx].sum_x += x;
                    fast_blobs[best_idx].sum_y += y;
                    fast_blobs[best_idx].count += 1;
                } else if (active_fast < MAX_BLOBS) {
                    fast_blobs[active_fast].sum_x = x;
                    fast_blobs[active_fast].sum_y = y;
                    fast_blobs[active_fast].count = 1;
                    active_fast++;
                }
            }
        }
    }


    struct Blob candidates[MAX_BLOBS];
    int cand_count = 0;
    for (int b = 0; b < active_fast; b++) {
        if (fast_blobs[b].count > FAST_MIN_COUNT) {
            candidates[cand_count].cord_x = (float)fast_blobs[b].sum_x / fast_blobs[b].count;
            candidates[cand_count].cord_y = (float)fast_blobs[b].sum_y / fast_blobs[b].count;
            candidates[cand_count].count = fast_blobs[b].count;
            cand_count++;
        }
    }


    if (cand_count < 3) {

        if (found_blobs && blind_frame_counter < MAX_BLIND_FRAMES) {
            blind_frame_counter++;
            result = last_good_result;
            xQueueOverwrite(dots_detection_queue, &result);
            ESP_LOGW(TAG, "Using predicted positions (blind frame %d)", blind_frame_counter);
            return result;
        } else {
            blind_frame_counter = 0;
            found_blobs = false;
            memset(&result, 0, sizeof(result));
            xQueueOverwrite(dots_detection_queue, &result);
            ESP_LOGW(TAG, "NOT ENOUGH DOTS (Need 3, found %d)", cand_count);
            return result;
        }
    }


    for (int i = 0; i < cand_count - 1; i++) {
        for (int j = i + 1; j < cand_count; j++) {
            if (candidates[j].cord_x < candidates[i].cord_x) {
                struct Blob tmp = candidates[i];
                candidates[i] = candidates[j];
                candidates[j] = tmp;
            }
        }
    }

    for (int i = 0; i < 3; i++) {
        result.blobs[i] = candidates[i];
    }


    float cx = width / 2.0f;
    float cy = height / 2.0f;
    for (int i = 0; i < 3; i++) {
        result.blobs[i].cord_x -= cx;
        result.blobs[i].cord_y = cy - result.blobs[i].cord_y;
    }


    blind_frame_counter = 0;
    last_good_result = result;
    found_blobs = true;


    if (xQueueOverwrite(dots_detection_queue, &result) == pdTRUE) {
        ESP_LOGI(TAG, "Data sent to queue successfully (fast path)");
    } else {
        ESP_LOGE(TAG, "Failed to send data to queue (fast path)");
    }

    ESP_LOGI(TAG, "LEFT DOT:   X=%.1f, Y=%.1f", result.blobs[0].cord_x, result.blobs[0].cord_y);
    ESP_LOGI(TAG, "CENTER DOT: X=%.1f, Y=%.1f", result.blobs[1].cord_x, result.blobs[1].cord_y);
    ESP_LOGI(TAG, "RIGHT DOT:  X=%.1f, Y=%.1f", result.blobs[2].cord_x, result.blobs[2].cord_y);

    return result;
}
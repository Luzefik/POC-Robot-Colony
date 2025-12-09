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


BlobResult process_image(camera_fb_t * fb) {
    BlobResult result = {0};
    float central_of_img_x = fb->width/2.0;
    float central_of_img_y = fb->height/2.0;
    float_t alpha = 0.8;  // coefficient for smoothing

    struct Blob blobs[MAX_BLOBS];
    int active_blobs = 0;

    for(int k=0; k<MAX_BLOBS; k++) blobs[k].count = 0;

    if (!fb) {
        ESP_LOGE(TAG, "No frame buffer provided");
        xQueueSend(dots_detection_queue, &result, portMAX_DELAY);
        return result;
    }


    for ( int i = 0; i < fb->len; i += 4) {

        uint8_t Y0 = fb->buf[i];
        uint8_t U  = fb->buf[i+1];
        uint8_t Y1 = fb->buf[i+2];
        uint8_t V  = fb->buf[i+3];

        // ONE condition check for shared U,V values
        if (V >= RED_V_THRESH && U < CHROMA_U_LOW) {

            // Process Y0 pixel
            if (Y0 > LUMA_THRESH) {
                int pixel_index = i / 2;  // Simpler math
                int x = pixel_index % fb->width;
                int y = pixel_index / fb->width;

                // Check if it belongs to an existing blob

                int best_blob_idx = -1;
                int min_dist_sq = 999999;

                for (int k = 0; k < active_blobs; k++) {
                    int dx = x - blobs[k].cord_x;
                    int dy = y - blobs[k].cord_y;
                    int dist_sq = dx * dx + dy * dy;
                    if (dist_sq < 100) {
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
                    blobs[best_blob_idx].cord_x = ((float_t)blobs[best_blob_idx].sum_x / blobs[best_blob_idx].count);
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

            // Process Y1 pixel
            if (Y1 > LUMA_THRESH) {
                int pixel_index = (i / 2) + 1;
                int x = pixel_index % fb->width;
                int y = pixel_index / fb->width;

                // Check if it belongs to an existing blob

                int best_blob_idx = -1;
                int min_dist_sq = 999999;

                for (int k = 0; k < active_blobs; k++) {
                    int dx = x - blobs[k].cord_x;
                    int dy = y - blobs[k].cord_y;
                    int dist_sq = dx * dx + dy * dy;
                    if (dist_sq < 100) {
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
                    blobs[best_blob_idx].cord_x = ((float_t)blobs[best_blob_idx].sum_x / blobs[best_blob_idx].count);
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
    /*
sorting blobs by size (count)
for (int i = 0; i < active_blobs - 1; i++) {
    for (int j = i + 1; j < active_blobs; j++) {
        if (blobs[j].count > blobs[i].count) {
            struct Blob temp = blobs[i];
            blobs[i] = blobs[j];
            blobs[j] = temp;
        }
    }
}
*/

// sorting to have min(diff(y)) to gurantee the line
int candidates_count = 0;
int candidate_indices[MAX_BLOBS];
int best_triplet[3] = {-1, -1, -1};
// int min_y_diff = 10000;
bool found_triplet = false;
float min_penalty = 10000.0;

for (int i = 0; i < active_blobs; i ++) {
    if (blobs[i].count > 10) {
        candidate_indices[candidates_count] = i;
            candidates_count++;
    }
}




for (int i=0; i < candidates_count-1; i ++) {
    for (int j = i + 1; j < candidates_count - 1; j++){
        for (int k = j+1; k < candidates_count; k ++){
            int indices[3] = { candidate_indices[i], candidate_indices[j], candidate_indices[k] };
            struct Blob p[3];
            for(int z=0; z<3; z++) p[z] = blobs[indices[z]];

            for (int a = 0; a <2; a ++) {
                for (int b = a+1; b < 3; b++) {
                    if (p[b].cord_x < p[a].cord_x) {
                                struct Blob temp = p[a]; p[a] = p[b]; p[b] = temp;
                                int temp_idx = indices[a]; indices[a] = indices[b]; indices[b] = temp_idx;
                            }
                        }
                    }


                        float y1 = p[0]. cord_y;
                        float y2 = p[1].cord_y;
                        float y3 = p[2].cord_y;

                        float min_y = fminf(y1, fminf(y2, y3));
                        float max_y = fmaxf(y1, fmaxf(y2, y3));
                        float error_y = max_y - min_y;
                        if (error_y > 20) continue; // too much y error

                        float gap_x_1 = p[1].cord_x - p[0].cord_x;
                        float gap_x_2 = p[2].cord_x - p[1].cord_x;
                        float symmetry_error = fabsf(gap_x_1 - gap_x_2);

                        // rating the triplet
                        float current_penalty = error_y + (symmetry_error * 1.5);

                        float total_width = p[2].cord_x - p[0].cord_x;
                        if (total_width < 20) current_penalty += 1000;

                        if (current_penalty < min_penalty){
                        min_penalty = current_penalty;
                        best_triplet[0] = indices[0];
                        best_triplet[1] = indices[1];
                        best_triplet[2] = indices[2];
                        found_triplet = true;
                        }
                    }
                }
            }

if (found_triplet && min_penalty < 55) {
    struct Blob sorted_blobs[3];

    sorted_blobs[0] = blobs[best_triplet[0]];
    sorted_blobs[1] = blobs[best_triplet[1]];
    sorted_blobs[2] = blobs[best_triplet[2]];

    blobs[0] = sorted_blobs[0];
    blobs[1] = sorted_blobs[1];
    blobs[2] = sorted_blobs[2];

    for (int i = 0; i < 3; i++) {
        blobs[i].cord_x = blobs[i].cord_x - central_of_img_x;
        blobs[i].cord_y = central_of_img_y - blobs[i].cord_y;
    }

    active_blobs = 3;
    static struct Blob prev_blobs[3];
    static bool is_initialized = false;


    if (!is_initialized) {
        for (int i = 0; i < 3; i++) {
            prev_blobs[i] = blobs[i];
        }
        is_initialized = true;
    } else {
        for (int i = 0; i < 3; i++) {
            blobs[i].cord_x = (blobs[i].cord_x * alpha) + (prev_blobs[i].cord_x * (1.0 - alpha));
            blobs[i].cord_y = (blobs[i].cord_y * alpha) + (prev_blobs[i].cord_y * (1.0 - alpha));


            prev_blobs[i] = blobs[i];
        }
    }
    }else
    {
    active_blobs = 0;
    ESP_LOGW(TAG, "Pattern failed validation. Penalty: %.2f", min_penalty);
}

/*
sorting by x coordinate top 3 blobs
for (int i = 0; i < 3; i++) {
    for (int j = i+1; j < 3; j++) {
        if (blobs[j].cord_x < blobs[i].cord_x) {
            struct Blob temp = blobs[i];
            blobs[i] = blobs[j];
            blobs[j] = temp;
        }
    }
}
*/

/*
if (active_blobs >= 3) {
        for (int i = 0; i < 2; i++) {
            for (int j = i + 1; j < 3; j++) {
                if (blobs[j].cord_x < blobs[i].cord_x) {
                    struct Blob temp = blobs[i];
                    blobs[i] = blobs[j];
                    blobs[j] = temp;
                }
            }
        }
    }
centring the blobs
for (int i = 0; i < 3; i ++) {
    blobs[i].cord_x = blobs[i].cord_x - central_of_img_x;
    blobs[i].cord_y = central_of_img_y - blobs[i].cord_y;
}
static int64_t last_log_time = 0;
    int64_t current_time = esp_timer_get_time() / 1000;
    if (current_time - last_log_time > 500) {
    last_log_time = current_time;
*/


if (active_blobs >= 3) {

    for (int i = 0; i < 3; i++) {
            result.blobs[i] = blobs[i];
        }


    blind_frame_counter = 0;

    last_good_result = result;
    found_blobs = true;

    xQueueReset(dots_detection_queue);

    if (xQueueSend(dots_detection_queue, &result, 0) == pdTRUE) {
            ESP_LOGI(TAG, "NEW DATA sent to queue");
        }

    ESP_LOGI(TAG, "LEFT DOT:   X=%.1f, Y=%.1f", blobs[0].cord_x, blobs[0].cord_y);
    ESP_LOGI(TAG, "CENTER DOT: X=%.1f, Y=%.1f", blobs[1].cord_x, blobs[1].cord_y);
    ESP_LOGI(TAG, "RIGHT DOT:  X=%.1f, Y=%.1f", blobs[2].cord_x, blobs[2].cord_y);

} else {
    if (found_blobs) {
        ESP_LOGI(TAG, "LOST DOTS!");
        if (found_blobs && blind_frame_counter < MAX_BLIND_FRAMES) {
            blind_frame_counter++;
            ESP_LOGI(TAG, "Using predicted positions (blind frame %d)", blind_frame_counter);
            blind_frame_counter++;
            result = last_good_result;

        } else {
            ESP_LOGW(TAG, "Max blind frames reached or tracking disabled. Stopping prediction.");
            found_blobs = false;
            memset(&result, 0, sizeof(BlobResult));
            ESP_LOGE(TAG, "LOST TARGET");
        }

    }
    ESP_LOGW(TAG, "NOT ENOUGH DOTS (Need 3, found %d)", active_blobs);
}


    if (xQueueSend(dots_detection_queue, &result, portMAX_DELAY) == pdTRUE) {
        ESP_LOGI(TAG, "Data sent to queue successfully");
    } else {
        ESP_LOGE(TAG, "Failed to send data to queue");
    }
    return result;
}
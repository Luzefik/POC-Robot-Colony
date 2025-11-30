
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "new_algo.h"
#include "esp_log.h"
#include "esp_log_timestamp.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "take_picture.h"

static const char *TAG = "dots_algo";

#define RED_V_THRESH 160
#define LUMA_THRESH 40
#define CHROMA_U_LOW 100
#define MAX_BLOBS 10


BlobResult process_image(camera_fb_t * fb) {
    BlobResult result = {0};
    float central_of_img_x = fb->width/2.0;
    float central_of_img_y = fb->height/2.0;


    struct Blob blobs[MAX_BLOBS];
    int active_blobs = 0;

    for(int k=0; k<MAX_BLOBS; k++) blobs[k].count = 0;

    if (!fb) {
        ESP_LOGE(TAG, "No frame buffer provided");
    }

    for ( int i = 0; i < fb->len; i +=4) {

        uint8_t Y0 = fb->buf[i];
        uint8_t U  = fb->buf[i+1];
        // uint8_t Y1 = fb->buf[i+2];
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
// // sorting blobs by size (count)
// for (int i = 0; i < active_blobs - 1; i++) {
//     for (int j = i + 1; j < active_blobs; j++) {
//         if (blobs[j].count > blobs[i].count) {
//             struct Blob temp = blobs[i];
//             blobs[i] = blobs[j];
//             blobs[j] = temp;
//         }
//     }
// }


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


// for (int i=0; i < candidates_count; i++) {
//     for (int j = i+1; j< candidates_count; j++){
//         for (int k = j + 1; k < candidates_count; k++) {
//             int idx1 = candidate_indices[i];
//             int idx2 = candidate_indices[j];
//             int idx3 = candidate_indices[k];

//             float y1 = blobs[idx1].cord_y;
//             float y2 = blobs[idx2].cord_y;
//             float y3 = blobs[idx3].cord_y;

//             float min_y = fminf(y1, fminf(y2, y3));
//             float max_y = fmaxf(y1, fmaxf(y2, y3));
//             float diff = max_y - min_y;
//             if (diff < min_y_diff) {
//                 min_y_diff = diff;
//                 best_triplet[0] = idx1;
//                 best_triplet[1] = idx2;
//                 best_triplet[2] = idx3;
//                 found_triplet = true;
//         }
//     }
// }}

// if (found_triplet) {
//     struct Blob final_blobs[3];
//     final_blobs[0] = blobs[best_triplet[0]];
//     final_blobs[1] = blobs[best_triplet[1]];
//     final_blobs[2] = blobs[best_triplet[2]];

//     blobs[0] = final_blobs[0];
//     blobs[1] = final_blobs[1];
//     blobs[2] = final_blobs[2];
//     active_blobs = 3;
//     } else {
//         if (candidates_count < 3) active_blobs = 0;
//     }


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

                        // int idx1 = candidate_indices[i];
                        // int idx2 = candidate_indices[j];
                        // int idx3 = candidate_indices[k];

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

if (found_triplet && min_penalty < 45) {
    struct Blob sorted_blobs[3];

    sorted_blobs[0] = blobs[best_triplet[0]];
    sorted_blobs[1] = blobs[best_triplet[1]];
    sorted_blobs[2] = blobs[best_triplet[2]];

    blobs[0] = sorted_blobs[0];
    blobs[1] = sorted_blobs[1];
    blobs[2] = sorted_blobs[2];

    active_blobs = 3;
} else {
    active_blobs = 0;
    ESP_LOGW(TAG, "Pattern failed validation. Penalty: %.2f", min_penalty);
}

//sorting by x coordinate top 3 blobs
// for (int i = 0; i < 3; i++) {
//     for (int j = i+1; j < 3; j++) {
//         if (blobs[j].cord_x < blobs[i].cord_x) {
//             struct Blob temp = blobs[i];
//             blobs[i] = blobs[j];
//             blobs[j] = temp;
//         }
//     }
// }




// if (active_blobs >= 3) {
//         for (int i = 0; i < 2; i++) {
//             for (int j = i + 1; j < 3; j++) {
//                 if (blobs[j].cord_x < blobs[i].cord_x) {
//                     struct Blob temp = blobs[i];
//                     blobs[i] = blobs[j];
//                     blobs[j] = temp;
//                 }
//             }
//         }
//     }


// centring the blobs
for (int i = 0; i < 3; i ++) {
    blobs[i].cord_x = blobs[i].cord_x - central_of_img_x;
    blobs[i].cord_y = central_of_img_y - blobs[i].cord_y;
}






// static int64_t last_log_time = 0;
//     int64_t current_time = esp_timer_get_time() / 1000;


    // if (current_time - last_log_time > 500) {
    // last_log_time = current_time;
if (active_blobs >= 3) {
    for (int i = 0; i < 3; i++) {
            result.blobs[i] = blobs[i];
        }

    ESP_LOGI(TAG, "LEFT DOT:   X=%.1f, Y=%.1f", blobs[0].cord_x, blobs[0].cord_y);
    ESP_LOGI(TAG, "CENTER DOT: X=%.1f, Y=%.1f", blobs[1].cord_x, blobs[1].cord_y);
    ESP_LOGI(TAG, "RIGHT DOT:  X=%.1f, Y=%.1f", blobs[2].cord_x, blobs[2].cord_y);

    int16_t gap_1_y = sqrt(pow(blobs[0].cord_x - blobs[1].cord_x, 2) + pow(blobs[0].cord_y - blobs[1].cord_y, 2));
    int16_t gap_2_y = sqrt(pow(blobs[1].cord_x - blobs[2].cord_x, 2) + pow(blobs[1].cord_y - blobs[2].cord_y, 2));
    int16_t gap_y = (gap_1_y + gap_2_y) / 2;

    ESP_LOGI(TAG, "Gap Y : %d", gap_y);
    ESP_LOGI(TAG, "Turn: %d", (int16_t)blobs[1].cord_x);
    float slope = blobs[2].cord_y - blobs[0].cord_y;
    ESP_LOGI(TAG, "Angle Slope: %.2f", slope);
} else {
    ESP_LOGW(TAG, "NOT ENOUGH DOTS (Need 3, found %d)", active_blobs);
}
   return result; }
// }
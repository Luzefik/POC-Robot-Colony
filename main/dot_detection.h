// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include "esp_camera.h"
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

struct Blob {
    float cord_x;
    float cord_y;
    int sum_x;
    int sum_y;
    int count;
};

typedef struct {
    struct Blob blobs[3];
}BlobResult;

BlobResult process_image(camera_fb_t * fb);
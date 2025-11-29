
#include "esp_camera.h"
#include <stdbool.h>

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
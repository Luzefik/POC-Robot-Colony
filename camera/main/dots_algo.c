#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <dots_algo.h>
#include "esp_log.h"
#include "jpeg_decoder.h"
#include "esp_heap_caps.h"
#include "esp_camera.h"
static const char *TAG = "dots_algo";

typedef struct {
    uint8_t *buf;
    size_t len;
    size_t width;
    size_t height;
} jpg_scale_t;

void detect_dots(camera_fb_t *fb) {
    if (!fb) {
        ESP_LOGE(TAG, "Invalid frame buffer (NULL)");
        return;
    }
    if (fb->format != PIXFORMAT_JPEG) {
        ESP_LOGE(TAG, "Frame is not JPEG format (format=%d)", fb->format);
        return;
    }
    detect_dots_in_frame(fb->buf, fb->len, fb->width, fb->height);
}

// decode jpg image to rgb565 format with scaling
static bool jpeg_to_rgb565(const uint8_t *jpeg, size_t jpeg_len, jpg_scale_t *out_img, esp_jpeg_image_scale_t scale)
{
    // Pre-allocate a large buffer (160x120 RGB565 = ~38.4KB for 1/4 scale)
    // Use larger size for safety: 50KB
    size_t output_size = 50 * 1024;
    out_img->buf = heap_caps_malloc(output_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!out_img->buf) {
        ESP_LOGE(TAG, "Failed to allocate output buffer (%d bytes)", output_size);
        return false;
    }

    // Decode JPEG directly with pre-allocated buffer
    esp_jpeg_image_cfg_t cfg = {
        .indata = (uint8_t *)jpeg,
        .indata_size = jpeg_len,
        .outbuf = out_img->buf,
        .outbuf_size = output_size,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = scale,
    };

    esp_jpeg_image_output_t img_output;
    esp_err_t err = esp_jpeg_decode(&cfg, &img_output);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "JPEG decode failed: %s", esp_err_to_name(err));
        heap_caps_free(out_img->buf);
        out_img->buf = NULL;
        return false;
    }

    out_img->width = img_output.width;
    out_img->height = img_output.height;
    out_img->len = img_output.output_len;

    return true;
}// Функція для розпакування 16-бітного кольору RGB565 в 24-бітний RGB
void rgb565_to_rgb(uint16_t rgb565, uint8_t* r, uint8_t* g, uint8_t* b) {
    *r = (uint8_t)(((rgb565 >> 11) & 0x1F) * 255 / 31);
    *g = (uint8_t)(((rgb565 >> 5) & 0x3F) * 255 / 63);
    *b = (uint8_t)((rgb565 & 0x1F) * 255 / 31);
}

// Функція для перевірки, чи є піксель "червоним"
int is_red(uint8_t r, uint8_t g, uint8_t b) {
    // Very relaxed thresholds for red-white LED detection
    // The LEDs are red with white edges, so we need to be more permissive
    return (r > 120 && g < 120 && b < 120 && r > g && r > b);
}

// Алгоритм пошуку в ширину (BFS) для знаходження зв'язаних компонентів (об'єктів)
void find_blobs_bfs(int x, int y, uint16_t* image_data, int* visited, int current_blob_id, Blob* blob, int width, int height) {
    Point *queue = heap_caps_malloc(width * height * sizeof(Point), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!queue) {
        ESP_LOGE(TAG, "Failed to allocate queue");
        return;
    }

    int head = 0, tail = 0;

    queue[tail++] = (Point){x, y};
    visited[y * width + x] = current_blob_id;

    long sum_x = 0;
    long sum_y = 0;
    int pixel_count = 0;

    while (head < tail) {
        Point p = queue[head++];
        sum_x += p.x;
        sum_y += p.y;
        pixel_count++;

        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;

                int nx = p.x + dx;
                int ny = p.y + dy;

                if (nx >= 0 && nx < width && ny >= 0 && ny < height && !visited[ny * width + nx]) {
                    uint16_t pixel = image_data[ny * width + nx];
                    uint8_t r, g, b;
                    rgb565_to_rgb(pixel, &r, &g, &b);

                    if (is_red(r, g, b)) {
                        visited[ny * width + nx] = current_blob_id;
                        queue[tail++] = (Point){nx, ny};
                    }
                }
            }
        }
    }

    if (pixel_count > 0) {
        blob->center.x = sum_x / pixel_count;
        blob->center.y = sum_y / pixel_count;
        blob->size = pixel_count;
        blob->id = current_blob_id;
    }

    free(queue);
}

// Функція для обчислення квадрату відстані між двома точками
double dist_sq(Point p1, Point p2) {
    return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
}

// Функція для перевірки колінеарності трьох точок
// Використовує площу трикутника. Якщо площа близька до нуля, точки колінеарні.
int are_collinear(Point p1, Point p2, Point p3, double tolerance) {
    long area = p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
    return labs(area) < tolerance;
}







void detect_dots_in_frame(const uint8_t *jpeg_data, size_t jpeg_len, int orig_width, int orig_height) {
    ESP_LOGI(TAG, "Starting detection: %dx%d JPEG (%d bytes)", orig_width, orig_height, jpeg_len);

    jpg_scale_t scaled_image = {0};
    if (!jpeg_to_rgb565(jpeg_data, jpeg_len, &scaled_image, JPEG_IMAGE_SCALE_1_4)) {
        ESP_LOGE(TAG, "Failed to decode JPEG");
        return;
    }

    int width = scaled_image.width;
    int height = scaled_image.height;
    uint16_t* image_data = (uint16_t*)scaled_image.buf;
    ESP_LOGI(TAG, "Decoded to %dx%d", width, height);

    // Виділення пам'яті
    int* visited = heap_caps_calloc(width * height, sizeof(int), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    Blob* blobs = heap_caps_malloc(100 * sizeof(Blob), MALLOC_CAP_8BIT);
    if (!visited || !blobs) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        if (visited) free(visited);
        if (blobs) free(blobs);
        free(scaled_image.buf);
        return;
    }

    // Пошук червоних
    int blob_count = 0;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (!visited[y * width + x]) {
                uint16_t pixel = image_data[y * width + x];
                uint8_t r, g, b;
                rgb565_to_rgb(pixel, &r, &g, &b);

                if (is_red(r, g, b)) {
                    if (blob_count < 100) {
                        find_blobs_bfs(x, y, image_data, visited,
                                       blob_count + 1, &blobs[blob_count],
                                       width, height);

                        if (blobs[blob_count].size > 5 && blobs[blob_count].size < 500) {
                            ESP_LOGI(TAG, "  Blob %d: center=(%d,%d) size=%d",
                                     blob_count, blobs[blob_count].center.x,
                                     blobs[blob_count].center.y, blobs[blob_count].size);
                            blob_count++;
                        }
                    }
                }
            }
        }
    }
    ESP_LOGI(TAG, "Found %d red objects", blob_count);

    // пошук трьох точок на одній лінії
    int found = 0;
    if (blob_count >= 3) {
        for (int i = 0; i < blob_count && !found; i++) {
            for (int j = i + 1; j < blob_count && !found; j++) {
                for (int k = j + 1; k < blob_count && !found; k++) {
                    double size_tolerance = 0.5;  // Increased from 0.2 to 0.5 (50% difference allowed)
                    if (abs(blobs[i].size - blobs[j].size) / (double)blobs[i].size > size_tolerance ||
                        abs(blobs[j].size - blobs[k].size) / (double)blobs[j].size > size_tolerance) {
                        continue;
                    }

                    Point p1 = blobs[i].center;
                    Point p2 = blobs[j].center;
                    Point p3 = blobs[k].center;

                    if (!are_collinear(p1, p2, p3, 100.0)) {
                        continue;
                    }

                    double d12_sq = dist_sq(p1, p2);
                    double d23_sq = dist_sq(p2, p3);
                    double dist_tolerance = 0.1;

                    if (fabs(sqrt(d12_sq) - sqrt(d23_sq)) < sqrt(d12_sq) * dist_tolerance) {
                        ESP_LOGI(TAG, "\n______FOUND THREE DOTS______");
                        ESP_LOGI(TAG, "Dot 1: (%d, %d) size=%d", p1.x, p1.y, blobs[i].size);
                        ESP_LOGI(TAG, "Dot 2: (%d, %d) size=%d", p2.x, p2.y, blobs[j].size);
                        ESP_LOGI(TAG, "Dot 3: (%d, %d) size=%d", p3.x, p3.y, blobs[k].size);
                        found = 1;
                    }
                }
            }
        }
    }

    if (!found) {
        ESP_LOGI(TAG, "Could not find three dots matching criteria");
    }

    free(image_data);
    free(visited);
    free(blobs);
}
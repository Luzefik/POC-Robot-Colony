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

static detection_result_t g_detection_result = {0};
static Point g_center_point = {0, 0};
static Point g_dots[3] = {{0, 0}, {0, 0}, {0, 0}};
static int g_dots_sizes[3] = {0, 0, 0};

void detect_dots(camera_fb_t *fb) {
    if (!fb) {
        return;
    }
    if (fb->format != PIXFORMAT_JPEG) {
        return;
    }
    if (fb->len < 4 || fb->buf[0] != 0xFF || fb->buf[1] != 0xD8) {
        return;
    }

    if (g_detection_result.rects) {
        free(g_detection_result.rects);
        g_detection_result.rects = NULL;
        g_detection_result.count = 0;
    }

    detect_dots_in_frame(fb->buf, fb->len, fb->width, fb->height);
}

detection_result_t* get_detection_result(void) {
    return &g_detection_result;
}

Point get_center_point(void) {
    return g_center_point;
}

Point* get_dots(void) {
    return g_dots;
}

int* get_dots_sizes(void) {
    return g_dots_sizes;
}

static bool jpeg_to_rgb565(const uint8_t *jpeg, size_t jpeg_len, jpg_scale_t *out_img, esp_jpeg_image_scale_t scale) {
    size_t output_size = 50 * 1024;
    out_img->buf = heap_caps_malloc(output_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!out_img->buf) {
        return false;
    }

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
        heap_caps_free(out_img->buf);
        out_img->buf = NULL;
        return false;
    }

    out_img->width = img_output.width;
    out_img->height = img_output.height;
    out_img->len = img_output.output_len;

    return true;
}

void rgb565_to_rgb(uint16_t rgb565, uint8_t* r, uint8_t* g, uint8_t* b) {
    *r = (uint8_t)(((rgb565 >> 11) & 0x1F) * 255 / 31);
    *g = (uint8_t)(((rgb565 >> 5) & 0x3F) * 255 / 63);
    *b = (uint8_t)((rgb565 & 0x1F) * 255 / 31);
}

int is_red(uint8_t r, uint8_t g, uint8_t b) {
    return (r > 120 && r > g + 50 && r > b + 50 && g < 120 && b < 120);
}

void find_blobs_bfs(int x, int y, uint16_t* image_data, int* visited, int current_blob_id,
                    Blob* blob, int width, int height) {
    Point *queue = heap_caps_malloc(width * height * sizeof(Point), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!queue) {
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

double dist_sq(Point p1, Point p2) {
    return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
}

int are_collinear(Point p1, Point p2, Point p3, double tolerance) {
    long area = p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
    return labs(area) < tolerance;
}

void detect_dots_in_frame(const uint8_t *jpeg_data, size_t jpeg_len, int orig_width, int orig_height) {
    jpg_scale_t scaled_image = {0};
    if (!jpeg_to_rgb565(jpeg_data, jpeg_len, &scaled_image, JPEG_IMAGE_SCALE_1_4)) {
        return;
    }

    int width = scaled_image.width;
    int height = scaled_image.height;
    uint16_t* image_data = (uint16_t*)scaled_image.buf;

    int center_x = width / 2;
    int center_y = height / 2;

    int* visited = heap_caps_calloc(width * height, sizeof(int), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    Blob* blobs = heap_caps_malloc(100 * sizeof(Blob), MALLOC_CAP_8BIT);
    if (!visited || !blobs) {
        if (visited) free(visited);
        if (blobs) free(blobs);
        heap_caps_free(scaled_image.buf);
        return;
    }

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

                        if (blobs[blob_count].size > 10 && blobs[blob_count].size < 200) {
                            blob_count++;
                        }
                    }
                }
            }
        }
    }

    g_detection_result.count = 0;
    g_center_point.x = 0;
    g_center_point.y = 0;
    memset(g_dots, 0, sizeof(g_dots));
    memset(g_dots_sizes, 0, sizeof(g_dots_sizes));

    if (blob_count >= 3) {
        for (int i = 0; i < blob_count; i++) {
            for (int j = i + 1; j < blob_count; j++) {
                for (int k = j + 1; k < blob_count; k++) {
                    double size_tolerance = 0.5;
                    if (abs(blobs[i].size - blobs[j].size) / (double)blobs[i].size > size_tolerance ||
                        abs(blobs[j].size - blobs[k].size) / (double)blobs[j].size > size_tolerance) {
                        continue;
                    }

                    Point p1 = blobs[i].center;
                    Point p2 = blobs[j].center;
                    Point p3 = blobs[k].center;

                    if (!are_collinear(p1, p2, p3, 500.0)) {
                        continue;
                    }

                    double d12_sq = dist_sq(p1, p2);
                    double d23_sq = dist_sq(p2, p3);
                    double dist_tolerance = 0.1;

                    if (fabs(sqrt(d12_sq) - sqrt(d23_sq)) < sqrt(d12_sq) * dist_tolerance) {
                        int center_abs_x = (p1.x + p3.x) / 2;
                        int center_abs_y = (p1.y + p3.y) / 2;

                        int center_rel_x = center_abs_x - center_x;
                        int center_rel_y = center_abs_y - center_y;

                        g_center_point.x = center_rel_x;
                        g_center_point.y = center_rel_y;

                        g_dots[0] = (Point){p1.x - center_x, p1.y - center_y};
                        g_dots[1] = (Point){p2.x - center_x, p2.y - center_y};
                        g_dots[2] = (Point){p3.x - center_x, p3.y - center_y};

                        g_dots_sizes[0] = blobs[i].size;
                        g_dots_sizes[1] = blobs[j].size;
                        g_dots_sizes[2] = blobs[k].size;

                        g_detection_result.count = 3;
                        g_detection_result.rects = malloc(3 * sizeof(dot_rect_t));

                        int radius = 10;
                        g_detection_result.rects[0] = (dot_rect_t){
                            .x = (p1.x - center_x) - radius,
                            .y = (p1.y - center_y) - radius,
                            .width = radius * 2,
                            .height = radius * 2
                        };
                        g_detection_result.rects[1] = (dot_rect_t){
                            .x = (p2.x - center_x) - radius,
                            .y = (p2.y - center_y) - radius,
                            .width = radius * 2,
                            .height = radius * 2
                        };
                        g_detection_result.rects[2] = (dot_rect_t){
                            .x = (p3.x - center_x) - radius,
                            .y = (p3.y - center_y) - radius,
                            .width = radius * 2,
                            .height = radius * 2
                        };

                        ESP_LOGI(TAG, "3 dots: center=(%d,%d) dot1=(%d,%d,%d) dot2=(%d,%d,%d) dot3=(%d,%d,%d)",
                                 center_rel_x, center_rel_y,
                                 g_dots[0].x, g_dots[0].y, g_dots_sizes[0],
                                 g_dots[1].x, g_dots[1].y, g_dots_sizes[1],
                                 g_dots[2].x, g_dots[2].y, g_dots_sizes[2]);
                        goto found_dots;
                    }
                }
            }
        }
    }

found_dots:
    heap_caps_free(scaled_image.buf);
    free(visited);
    free(blobs);
}
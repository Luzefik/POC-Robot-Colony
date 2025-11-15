#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <dots_algo.h>
#include "esp_heap_caps.h"
#include "esp_camera.h"
#include "esp_log.h"

static detection_result_t g_detection_result = {0};
static Point g_center_point = {0, 0};
static Point g_dots[3] = {{0, 0}, {0, 0}, {0, 0}};
static int g_dots_sizes[3] = {0, 0, 0};
static double g_last_known_d_total = 0.0; 
static bool g_is_tracking = false;
static Point g_last_known_B = {0, 0}; // абсолютні координати останньої центральної точки (p_B)
void rgb565_to_rgb(uint16_t rgb565, uint8_t* r, uint8_t* g, uint8_t* b);
int is_red(uint8_t r, uint8_t g, uint8_t b);
void find_blobs_bfs(int start_x, int start_y, uint16_t* image_data, int* visited, int current_blob_id,
                    Blob* blob, int width, int height);
double dist_sq(Point p1, Point p2);
int are_collinear(Point p1, Point p2, Point p3, double tolerance);

void detect_dots(camera_fb_t *fb) {
    if (!fb) {
        ESP_LOGW("DOTS", "fb is NULL!");
        return;
    }

    if (fb->format != PIXFORMAT_RGB565) {
        ESP_LOGW("DOTS", "Unsupported format=%d (need RGB565)", fb->format);
        return;
    }

    ESP_LOGI("DOTS", "=== Processing RGB565: %d bytes, %dx%d ===", fb->len, fb->width, fb->height);

    uint16_t *rgb_data = (uint16_t*)fb->buf;
    int width = fb->width;
    int height = fb->height;
    int center_x = width / 2;
    int center_y = height / 2;

    int* visited = heap_caps_calloc(width * height, sizeof(int), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    Blob* blobs = heap_caps_malloc(100 * sizeof(Blob), MALLOC_CAP_8BIT);
    if (!visited || !blobs) {
        ESP_LOGE("DOTS", "Memory allocation failed");
        if (visited) free(visited);
        if (blobs) free(blobs);
        return;
    }

    int blob_count = 0;
    int x_start = 0, y_start = 0, x_end = width, y_end = height;
    const int SEARCH_WINDOW_SIZE = 150; // Розмір вікна 150x150

    if (g_is_tracking) {
        ESP_LOGI("DOTS", "Режим ВІДСТЕЖЕННЯ навколо (%d, %d)", g_last_known_B.x, g_last_known_B.y);

        x_start = g_last_known_B.x - SEARCH_WINDOW_SIZE / 2;
        y_start = g_last_known_B.y - SEARCH_WINDOW_SIZE / 2;
        x_end = x_start + SEARCH_WINDOW_SIZE;
        y_end = y_start + SEARCH_WINDOW_SIZE;

        // Обмежуємо вікно, щоб не вийти за межі кадру
        if (x_start < 0) x_start = 0;
        if (y_start < 0) y_start = 0;
        if (x_end > width) x_end = width;
        if (y_end > height) y_end = height;
    } else {
        ESP_LOGI("DOTS", "Режим ПОШУКУ (повний кадр)");
    }
    // ------------------------------------

    // Шукаємо всі червоні об'єкти у визначеному вікні
    for (int y = y_start; y < y_end; y++) {
        for (int x = x_start; x < x_end; x++) {
            if (!visited[y * width + x]) {
                uint16_t pixel = rgb_data[y * width + x];
                uint8_t r, g, b;
                rgb565_to_rgb(pixel, &r, &g, &b);

                if (is_red(r, g, b)) {
                    if (blob_count < 100) {
                        find_blobs_bfs(x, y, rgb_data, visited,
                                       blob_count + 1, &blobs[blob_count],
                                       width, height);

                        // Фільтруємо об'єкти за розміром
                        if (blobs[blob_count].size > 10 && blobs[blob_count].size < 200) {
                            blob_count++;
                        }
                    }
                }
            }
        }
    }

    ESP_LOGI("DOTS", "Found %d blobs (after filtering)", blob_count);
    for (int i = 0; i < blob_count && i < 5; i++) {
        ESP_LOGI("DOTS", "  Blob %d: center(%d,%d) size=%d",
                 i + 1, blobs[i].center.x, blobs[i].center.y, blobs[i].size);
    }

    int found = 0;
    if (blob_count >= 3) {
        for (int i = 0; i < blob_count; i++) {
            for (int j = i + 1; j < blob_count; j++) {
                for (int k = j + 1; k < blob_count; k++) {

                    // Перевірка на приблизно однаковий розмір (20%)
                    double size_tolerance = 0.2;
                    if (abs(blobs[i].size - blobs[j].size) / (double)blobs[i].size > size_tolerance ||
                        abs(blobs[j].size - blobs[k].size) / (double)blobs[j].size > size_tolerance) {
                        continue;
                    }

                    Point p1 = blobs[i].center;
                    Point p2 = blobs[j].center;
                    Point p3 = blobs[k].center;

                    // 1. Рахуємо квадрати відстаней (швидко)
                    double d12_sq = dist_sq(p1, p2);
                    double d23_sq = dist_sq(p2, p3);
                    double d13_sq = dist_sq(p1, p3);

                    // 2. Рахуємо реальні відстані (повільно, але потрібно для порівняння)
                    double d12 = sqrt(d12_sq);
                    double d23 = sqrt(d23_sq);
                    double d13 = sqrt(d13_sq);

                    // 3. Сортуємо відстані, щоб знайти дві менші (d_side1, d_side2)
                    //    і одну найбільшу (d_total)
                    double d_side1, d_side2, d_total;
                    if (d12 > d23 && d12 > d13) { // d12 - найбільша
                        d_total = d12; d_side1 = d23; d_side2 = d13;
                    } else if (d23 > d12 && d23 > d13) { // d23 - найбільша
                        d_total = d23; d_side1 = d12; d_side2 = d13;
                    } else { // d13 - найбільша
                        d_total = d13; d_side1 = d12; d_side2 = d23;
                    }

                    // 4. Встановлюємо похибку (3% - суворіше, менше хибних)
                    double tolerance = 0.03;
                    bool sides_are_equal = fabs(d_side1 - d_side2) < (d_side1 * tolerance);

                    bool are_collinear_geo = fabs(d_total - (d_side1 + d_side2)) < (d_total * tolerance);

                    if (sides_are_equal && are_collinear_geo) {

                        // --- ДОДАНО: Фільтр за розміром (Scale Gating) ---
                        if (g_last_known_d_total > 0.0) {
                            double size_change_ratio = d_total / g_last_known_d_total;
                            if (size_change_ratio > 1.3 || size_change_ratio < 0.7) {
                                ESP_LOGW("DOTS", "Розкид великий. відхилено: new_total=%.1f, old_total=%.1f", d_total, g_last_known_d_total);
                                continue;
                            }
                        }

                        ESP_LOGI("DOTS", " Знайдено патерн 1:1 Блоби [%d, %d, %d]", i, j, k);
                        ESP_LOGI("DOTS", "   d_side1=%.1f, d_side2=%.1f, d_total=%.1f",
                                 d_side1, d_side2, d_total);

                        Point p_A, p_B, p_C;

                        if (d_total == d12) {
                            p_B = p3; p_A = p1; p_C = p2;
                        } else if (d_total == d23) {
                            p_B = p1; p_A = p2; p_C = p3;
                        } else {
                            p_B = p2; p_A = p1; p_C = p3;
                        }

                        if (p_A.x > p_C.x) {
                            Point temp = p_A;
                            p_A = p_C;
                            p_C = temp;
                        }

                        ESP_LOGI("DOTS", "   A(%d,%d) B(%d,%d) C(%d,%d)",
                                 p_A.x, p_A.y, p_B.x, p_B.y, p_C.x, p_C.y);

                        // Зберігаємо АБСОЛЮТНУ позицію p_B для наступного кадру
                        g_last_known_B = p_B;
                        g_is_tracking = true; // Ми "захопили" ціль

                        // Центральна точка патерну - це p_B
                        g_center_point.x = p_B.x - center_x;
                        g_center_point.y = p_B.y - center_y;

                        g_dots[0] = (Point){p_A.x - center_x, p_A.y - center_y};
                        g_dots[1] = (Point){p_B.x - center_x, p_B.y - center_y};
                        g_dots[2] = (Point){p_C.x - center_x, p_C.y - center_y};

                        // Зберігаємо розміри (потрібно знайти правильні індекси)
                        if (p_B.x == blobs[i].center.x && p_B.y == blobs[i].center.y) {
                            g_dots_sizes[1] = blobs[i].size;
                            if (p_A.x == blobs[j].center.x) {
                                g_dots_sizes[0] = blobs[j].size;
                                g_dots_sizes[2] = blobs[k].size;
                            } else {
                                g_dots_sizes[0] = blobs[k].size;
                                g_dots_sizes[2] = blobs[j].size;
                            }
                        } else if (p_B.x == blobs[j].center.x && p_B.y == blobs[j].center.y) {
                            g_dots_sizes[1] = blobs[j].size;
                            if (p_A.x == blobs[i].center.x) {
                                g_dots_sizes[0] = blobs[i].size;
                                g_dots_sizes[2] = blobs[k].size;
                            } else {
                                g_dots_sizes[0] = blobs[k].size;
                                g_dots_sizes[2] = blobs[i].size;
                            }
                        } else {
                            g_dots_sizes[1] = blobs[k].size;
                            if (p_A.x == blobs[i].center.x) {
                                g_dots_sizes[0] = blobs[i].size;
                                g_dots_sizes[2] = blobs[j].size;
                            } else {
                                g_dots_sizes[0] = blobs[j].size;
                                g_dots_sizes[2] = blobs[i].size;
                            }
                        }

                        // Оновлюємо останній підтверджений розмір
                        g_last_known_d_total = d_total;

                        g_detection_result.count = 3;
                        found = 1;
                        goto cleanup;
                    }
                }
            }
        }
    }

cleanup:
    if (!found) {
        ESP_LOGW("DOTS", "No valid 3-dot pattern found");
        // Повільно "забуваємо" старий розмір для можливості перезахоплення
        g_last_known_d_total *= 0.9;
        // Втратили ціль - на наступному кадрі шукаємо по всьому екрану
        g_is_tracking = false;
    }

    free(visited);
    free(blobs);
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

detection_data_t get_detection_data(void) {
    detection_data_t result;
    result.count = g_detection_result.count;
    for (int i = 0; i < 3; i++) {
        result.data[i] = g_dots[i];
        result.sizes[i] = g_dots_sizes[i];
    }
    return result;
}

// Функція для розпакування 16-бітного кольору RGB565 в 24-бітний RGB
void rgb565_to_rgb(uint16_t rgb565, uint8_t* r, uint8_t* g, uint8_t* b) {
    *r = (uint8_t)(((rgb565 >> 11) & 0x1F) * 255 / 31);
    *g = (uint8_t)(((rgb565 >> 5) & 0x3F) * 255 / 63);
    *b = (uint8_t)((rgb565 & 0x1F) * 255 / 31);
}

int is_red(uint8_t r, uint8_t g, uint8_t b) {
    // СУВОРИЙ фільтр як у робочому алгоритмі:
    // Червоний яскравий (>180), зелений і синій темні (<80)
    return (r > 180 && g < 80 && b < 80);
}

/* Функція BFS для пошуку бліб */
void find_blobs_bfs(int start_x, int start_y, uint16_t* image_data, int* visited, int current_blob_id,
                    Blob* blob, int width, int height) {
    // Використовуємо фіксований розмір черги (достатньо для блобів розміром до 10000 пікселів)
    #define MAX_QUEUE_SIZE 10000
    static Point queue[MAX_QUEUE_SIZE];

    int head = 0, tail = 0;
    queue[tail++] = (Point){start_x, start_y};
    visited[start_y * width + start_x] = current_blob_id;

    long sum_x = 0;
    long sum_y = 0;
    int pixel_count = 0;

    while (head < tail && tail < MAX_QUEUE_SIZE) {
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

                    if (is_red(r, g, b) && tail < MAX_QUEUE_SIZE) {
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

    #undef MAX_QUEUE_SIZE
}

double dist_sq(Point p1, Point p2) {
    return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
}

int are_collinear(Point p1, Point p2, Point p3, double tolerance) {
    long area = p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
    return labs(area) < tolerance;
}

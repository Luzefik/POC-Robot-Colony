/**
 * @file dot_detection.c
 * @brief Пошук трьох червоних ліхтариків у YUV422 (YUYV) кадрі.
 *
 * Як працює (докладно: main/DOT_DETECTION.md):
 *   1. Фільтр кольору: V >= 150 та U < 100 виділяють червоне,
 *      Y > 50 відсіює темне. Яскравість (Y) береться через горизонтальний
 *      фільтр Гауса на 5 точок - прибирає шум сенсора.
 *   2. Кластеризація: сусідні "червоні" пікселі збираються у плями.
 *   3. Геометрична перевірка трійки: майже горизонтальна лінія, достатня
 *      ширина, центральна точка приблизно посередині. Під час трекінгу
 *      трійка ще й не може різко стрибнути або змінити розмір - саме це
 *      не дає фолловеру "перечепитись" на сторонній червоний об'єкт.
 *   4. Згладжування координат: EMA з alpha = 0.8 (прибирає тремтіння).
 *
 * Дебаг наживо: /mask показує, які пікселі проходять пороги,
 * / показує рамки навколо знайденої трійки, /log - причини відмов.
 */

#include "dot_detection.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "blob_detect";

extern QueueHandle_t dots_detection_queue;

/* ── Налаштування (тюнити тут; перевіряти наживо через /mask і /log) ───── */

// Пороги "червоності" пікселя (простір YUV)
#define RED_V_MIN 150 // канал V (Cr): високий = червоне
#define RED_U_MAX 100 // канал U (Cb): низький = не синє/фіолетове
#define LUMA_MIN 50   // канал Y: мінімальна яскравість (відсіює темне)

// Кластеризація плям
#define MAX_BLOBS 10      // скільки плям трекаємо одночасно
#define MIN_BLOB_PIXELS 5 // менші плями - шум, викидаємо
#define CLUSTER_RADIUS 50 // пікселі ближче цього зливаються в одну пляму

// Прискорення сканування
#define SCAN_STEP 2 // крок по пікселях (1 = кожен, 2 = через один)

// Геометрична перевірка трійки ліхтариків
#define GEO_MAX_DY 20.0f     // мінімальний допуск по вертикалі, px
#define GEO_MAX_DY_FRAC 0.3f // ...і росте з шириною трійки: якщо камера
                             // або планка нахилені, лінія теж нахилена
#define GEO_MIN_WIDTH 20.0f  // трійка вужча за це - випадкові плями
#define GEO_SYM_TOL 0.35f    // центр посередині: |d_лів - d_прав| <= tol * ширина

// Захист від "перестрибування" на чужий об'єкт під час трекінгу
#define TRACK_MAX_JUMP_PX 80.0f // ціль не може телепортнутись за кадр
#define TRACK_SCALE_MIN 0.6f    // і не може різко змінити розмір
#define TRACK_SCALE_MAX 1.6f
#define MAX_BLIND_FRAMES 5 // скільки кадрів тримаємо стару позицію при збої

// Згладжування виходу
#define EMA_ALPHA 0.8f // вага нового виміру (0.8 нове + 0.2 старе)

/* ── Внутрішні типи і стан ─────────────────────────────────────────────── */

typedef struct {
    long sum_x;
    long sum_y;
    int count;
} BlobAccumulator;

typedef struct {
    float x; /* raw image coordinates (origin top-left) */
    float y;
    int count;
} Candidate;

static int blind_frame_count = 0;
static BlobResult last_good_result = {0};
static bool has_valid_track = false;

/* Last confirmed triple in raw image coordinates (for gating and EMA). */
static float track_cx, track_cy, track_width;
static Candidate track_dots[3];

/* ── Helpers ───────────────────────────────────────────────────────────── */

static inline bool is_red_pixel(uint8_t y, uint8_t u, uint8_t v) {
    return (v >= RED_V_MIN) && (u < RED_U_MAX) && (y > LUMA_MIN);
}

/* Horizontal 5-tap Gaussian (1 4 6 4 1)/16 over the Y channel of one YUYV
 * row. Applied only to sampled pixels, so the frame itself stays intact
 * (the same buffer may be JPEG-encoded by the debug web stream). */
static inline uint8_t blurred_luma(const uint8_t *row, int x, int width) {
    static const int w[5] = {1, 4, 6, 4, 1};
    int sum = 0;
    for (int i = -2; i <= 2; i++) {
        int xi = x + i;
        if (xi < 0)
            xi = 0;
        else if (xi >= width)
            xi = width - 1;
        sum += w[i + 2] * row[xi * 2];
    }
    return (uint8_t)(sum / 16);
}

static int find_nearest_blob(BlobAccumulator *blobs, int count, int x, int y) {
    int best_idx = -1;
    long best_dist = (long)CLUSTER_RADIUS * CLUSTER_RADIUS;

    for (int i = 0; i < count; i++) {
        if (blobs[i].count == 0)
            continue;

        int cx = blobs[i].sum_x / blobs[i].count;
        int cy = blobs[i].sum_y / blobs[i].count;
        int dx = x - cx;
        int dy = y - cy;

        if (abs(dx) > CLUSTER_RADIUS || abs(dy) > CLUSTER_RADIUS)
            continue;

        long dist_sq = (long)dx * dx + (long)dy * dy;
        if (dist_sq < best_dist) {
            best_dist = dist_sq;
            best_idx = i;
        }
    }
    return best_idx;
}

static void add_pixel_to_blobs(BlobAccumulator *blobs, int *active_count, int x,
                               int y) {
    int idx = find_nearest_blob(blobs, *active_count, x, y);

    if (idx >= 0) {
        blobs[idx].sum_x += x;
        blobs[idx].sum_y += y;
        blobs[idx].count += 1;
    } else if (*active_count < MAX_BLOBS) {
        idx = (*active_count)++;
        blobs[idx].sum_x = x;
        blobs[idx].sum_y = y;
        blobs[idx].count = 1;
    }
    // Else: too many blobs, pixel ignored
}

/* Reject counters for the last select_triple() call - printed (throttled)
 * when candidates exist but no triple passes, so threshold tuning is based
 * on data instead of guessing. */
static struct {
    int dy, width, sym, scale, jump;
} g_rejects;

/* Pick the candidate triple that best matches the LED reference geometry.
 * Returns true and fills out[3] (ordered left/center/right) on success. */
static bool select_triple(const Candidate *c, int n, Candidate out[3]) {
    float best_score = FLT_MAX;
    bool found = false;

    memset(&g_rejects, 0, sizeof(g_rejects));

    for (int i = 0; i < n - 2; i++) {
        for (int j = i + 1; j < n - 1; j++) {
            for (int k = j + 1; k < n; k++) {
                const Candidate *t[3] = {&c[i], &c[j], &c[k]};

                /* Order by x: left, center, right. */
                const Candidate *tmp;
                if (t[0]->x > t[1]->x) { tmp = t[0]; t[0] = t[1]; t[1] = tmp; }
                if (t[1]->x > t[2]->x) { tmp = t[1]; t[1] = t[2]; t[2] = tmp; }
                if (t[0]->x > t[1]->x) { tmp = t[0]; t[0] = t[1]; t[1] = tmp; }

                /* Sufficient total width. */
                float width = t[2]->x - t[0]->x;
                if (width < GEO_MIN_WIDTH) {
                    g_rejects.width++;
                    continue;
                }

                /* Approximately horizontal: allowance grows with width,
                 * because a rolled camera or tilted LED bar slopes the
                 * whole line. */
                float ymin = fminf(t[0]->y, fminf(t[1]->y, t[2]->y));
                float ymax = fmaxf(t[0]->y, fmaxf(t[1]->y, t[2]->y));
                float max_dy = fmaxf(GEO_MAX_DY, GEO_MAX_DY_FRAC * width);
                if (ymax - ymin > max_dy) {
                    g_rejects.dy++;
                    continue;
                }

                /* Center dot roughly in the middle (1:1 spacing). */
                float d_left = t[1]->x - t[0]->x;
                float d_right = t[2]->x - t[1]->x;
                if (fabsf(d_left - d_right) > GEO_SYM_TOL * width) {
                    g_rejects.sym++;
                    continue;
                }

                float score;
                if (has_valid_track) {
                    /* Gate against the last confirmed detection: the leader
                     * cannot teleport or change apparent size in one frame. */
                    float scale = width / track_width;
                    if (scale < TRACK_SCALE_MIN || scale > TRACK_SCALE_MAX) {
                        g_rejects.scale++;
                        continue;
                    }

                    float jump = hypotf(t[1]->x - track_cx, t[1]->y - track_cy);
                    if (jump > TRACK_MAX_JUMP_PX) {
                        g_rejects.jump++;
                        continue;
                    }
                    score = jump;
                } else {
                    /* Acquisition: prefer the strongest (brightest) triple. */
                    score = -(float)(t[0]->count + t[1]->count + t[2]->count);
                }

                if (score < best_score) {
                    best_score = score;
                    out[0] = *t[0];
                    out[1] = *t[1];
                    out[2] = *t[2];
                    found = true;
                }
            }
        }
    }
    return found;
}

/* Snapshot of the last published result for the stream overlay. */
static portMUX_TYPE g_last_mux = portMUX_INITIALIZER_UNLOCKED;
static BlobResult g_last_published;

BlobResult dot_detection_get_last(void) {
    taskENTER_CRITICAL(&g_last_mux);
    BlobResult copy = g_last_published;
    taskEXIT_CRITICAL(&g_last_mux);
    return copy;
}

static BlobResult send_result(BlobResult *result) {
    taskENTER_CRITICAL(&g_last_mux);
    g_last_published = *result;
    taskEXIT_CRITICAL(&g_last_mux);

    if (dots_detection_queue &&
        xQueueOverwrite(dots_detection_queue, result) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send to queue");
    }
    return *result;
}

void dot_detection_paint_mask(camera_fb_t *fb) {
    if (!fb || !fb->buf || fb->format != PIXFORMAT_YUV422)
        return;

    const int width = fb->width;
    const int height = fb->height;
    uint8_t *buf = fb->buf;

    for (int y = 0; y < height; y += SCAN_STEP) {
        size_t row_start = (size_t)y * width * 2;
        if (row_start + (size_t)width * 2 > fb->len)
            break;
        uint8_t *row = buf + row_start;

        for (int x = 0; x < width; x += SCAN_STEP) {
            int pair_base = (x & ~1) * 2;
            uint8_t U = row[pair_base + 1];
            uint8_t V = row[pair_base + 3];
            if (V < RED_V_MIN || U >= RED_U_MAX)
                continue;

            if (is_red_pixel(blurred_luma(row, x, width), U, V)) {
                row[x * 2] = 255; // bright green marker
                row[pair_base + 1] = 0;
                row[pair_base + 3] = 0;
            }
        }
    }
}

/* ── Main processing function ──────────────────────────────────────────── */

/**
 * @param fb Camera frame buffer (YUV422/YUYV: [Y0][U][Y1][V] per pixel pair)
 * @return BlobResult; valid=true only when a geometrically consistent LED
 *         triple was found (or briefly held from the previous frames).
 */
BlobResult process_image(camera_fb_t *fb) {
    BlobResult result = {0};

    if (!fb || !fb->buf) {
        ESP_LOGE(TAG, "Invalid frame buffer");
        return send_result(&result);
    }

    const int width = fb->width;
    const int height = fb->height;
    const uint8_t *buf = fb->buf;
    const size_t buf_len = fb->len;

    /* ── Pass 1: scan for red pixels and cluster into blobs ────────────── */

    BlobAccumulator blobs[MAX_BLOBS] = {0};
    int active_blobs = 0;

    for (int y = 0; y < height; y += SCAN_STEP) {
        size_t row_start = (size_t)y * width * 2;
        if (row_start + (size_t)width * 2 > buf_len)
            break;
        const uint8_t *row = buf + row_start;

        for (int x = 0; x < width; x += SCAN_STEP) {
            int pair_base = (x & ~1) * 2; // U/V shared between the pixel pair

            uint8_t U = row[pair_base + 1];
            uint8_t V = row[pair_base + 3];
            if (V < RED_V_MIN || U >= RED_U_MAX)
                continue; // cheap chroma reject before the blur

            uint8_t Y = blurred_luma(row, x, width);
            if (is_red_pixel(Y, U, V))
                add_pixel_to_blobs(blobs, &active_blobs, x, y);
        }
    }

    /* ── Pass 2: accumulators → candidates (size filter) ───────────────── */

    Candidate candidates[MAX_BLOBS];
    int candidate_count = 0;

    for (int i = 0; i < active_blobs; i++) {
        if (blobs[i].count >= MIN_BLOB_PIXELS) {
            candidates[candidate_count].x = (float)blobs[i].sum_x / blobs[i].count;
            candidates[candidate_count].y = (float)blobs[i].sum_y / blobs[i].count;
            candidates[candidate_count].count = blobs[i].count;
            candidate_count++;
        }
    }

    /* ── Pass 3: geometric validation and triple selection ─────────────── */

    Candidate triple[3];
    bool found = candidate_count >= 3 && select_triple(candidates, candidate_count, triple);

    if (!found && candidate_count >= 3) {
        /* Blobs exist but no triple passed - explain why, once a second. */
        static int64_t last_diag_us;
        int64_t now = esp_timer_get_time();
        if (now - last_diag_us > 1000000) {
            last_diag_us = now;
            ESP_LOGW(TAG,
                     "%d blobs, no triple: rejects dy=%d width=%d sym=%d "
                     "scale=%d jump=%d",
                     candidate_count, g_rejects.dy, g_rejects.width,
                     g_rejects.sym, g_rejects.scale, g_rejects.jump);
            for (int i = 0; i < candidate_count && i < 5; i++)
                ESP_LOGW(TAG, "  blob %d: (%.0f, %.0f) %d px", i,
                         candidates[i].x, candidates[i].y,
                         candidates[i].count);
        }
    }

    if (!found) {
        /* Hold the last result for a few frames to smooth short dropouts. */
        if (has_valid_track && blind_frame_count < MAX_BLIND_FRAMES) {
            blind_frame_count++;
            ESP_LOGD(TAG, "Blind frame %d/%d - using last position",
                     blind_frame_count, MAX_BLIND_FRAMES);
            return send_result(&last_good_result);
        }

        if (has_valid_track)
            ESP_LOGW(TAG, "Tracking lost (%d candidate blobs)", candidate_count);
        blind_frame_count = 0;
        has_valid_track = false;
        return send_result(&result); // valid = false
    }

    /* ── Pass 4: EMA smoothing alpha = 0.8 ────────────────────── */

    if (has_valid_track) {
        for (int i = 0; i < 3; i++) {
            triple[i].x = EMA_ALPHA * triple[i].x + (1.0f - EMA_ALPHA) * track_dots[i].x;
            triple[i].y = EMA_ALPHA * triple[i].y + (1.0f - EMA_ALPHA) * track_dots[i].y;
        }
    }

    for (int i = 0; i < 3; i++)
        track_dots[i] = triple[i];
    track_cx = triple[1].x;
    track_cy = triple[1].y;
    track_width = triple[2].x - triple[0].x;

    /* ── Pass 5: image-centered output ─────────────────────────────────── */

    float center_x = width / 2.0f;
    float center_y = height / 2.0f;

    result.valid = true;
    for (int i = 0; i < 3; i++) {
        result.dots[i].x = triple[i].x - center_x; // right positive
        result.dots[i].y = center_y - triple[i].y; // up positive
        result.dots[i].count = triple[i].count;
    }
    result.spacing_px = track_width;

    blind_frame_count = 0;
    last_good_result = result;
    if (!has_valid_track)
        ESP_LOGI(TAG, "Triple ACQUIRED: center (%.0f, %.0f), width %.0f px",
                 result.dots[1].x, result.dots[1].y, result.spacing_px);
    has_valid_track = true;

    ESP_LOGD(TAG, "Triple: L(%.0f,%.0f) C(%.0f,%.0f) R(%.0f,%.0f) w=%.0f",
             result.dots[0].x, result.dots[0].y, result.dots[1].x,
             result.dots[1].y, result.dots[2].x, result.dots[2].y,
             result.spacing_px);

    return send_result(&result);
}

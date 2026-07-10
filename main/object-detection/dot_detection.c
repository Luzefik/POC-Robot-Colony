/**
 * @file dot_detection.c
 * @brief Red LED triple detection for ESP32-CAM (YUV422/YUYV frames).
 *
 * Pipeline ("Column of Ground Robots", section III.A):
 *   1. Color space filtering: V >= 150, U < 100 isolate the red LEDs,
 *      Y > 50 ensures brightness. Y is sampled through a horizontal 5-tap
 *      Gaussian to suppress sensor noise.
 *   2. Blob detection: nearest-neighbour clustering of accepted pixels.
 *   3. Geometric validation: the triple must be nearly horizontal
 *      (dy < 20 px), wide enough (> 20 px) and symmetric. While tracking it
 *      must also stay close to the last confirmed position and scale, which
 *      stops the follower from locking onto an unrelated red object.
 *   4. Smoothing: EMA (alpha = 0.8) on the output coordinates.
 */

#include "dot_detection.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "blob_detect";

extern QueueHandle_t dots_detection_queue;

/* ── Configuration ─────────────────────────────────────────────────────── */

// Red detection thresholds (YUV space)
#define RED_V_MIN 150 // V (Cr) channel: high = red
#define RED_U_MAX 100 // U (Cb) channel: low = not blue/purple
#define LUMA_MIN 50   // Y channel: minimum brightness

// Blob detection parameters
#define MAX_BLOBS 10      // Maximum blobs to track simultaneously
#define MIN_BLOB_PIXELS 5 // Minimum pixels to consider a valid blob
#define CLUSTER_RADIUS 50 // Max distance (pixels) to merge into same blob

// Scan optimization
#define SCAN_STEP 2 // Pixel skip (1=full, 2=half resolution scan)

// Geometric validation of the LED triple
#define GEO_MAX_DY 20.0f   // max vertical spread inside the triple, px
#define GEO_MIN_WIDTH 20.0f // min horizontal width of the triple, px
#define GEO_SYM_TOL 0.35f  // |d_left - d_right| <= tol * width

// Temporal consistency while tracking
#define TRACK_MAX_JUMP_PX 80.0f // max center displacement between frames
#define TRACK_SCALE_MIN 0.6f    // accepted width change between frames
#define TRACK_SCALE_MAX 1.6f
#define MAX_BLIND_FRAMES 5 // frames to keep last result when detection fails

// Output smoothing
#define EMA_ALPHA 0.8f // weight of the new measurement

/* ── Internal types and state ──────────────────────────────────────────── */

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

/* Pick the candidate triple that best matches the LED reference geometry.
 * Returns true and fills out[3] (ordered left/center/right) on success. */
static bool select_triple(const Candidate *c, int n, Candidate out[3]) {
    float best_score = FLT_MAX;
    bool found = false;

    for (int i = 0; i < n - 2; i++) {
        for (int j = i + 1; j < n - 1; j++) {
            for (int k = j + 1; k < n; k++) {
                const Candidate *t[3] = {&c[i], &c[j], &c[k]};

                /* Order by x: left, center, right. */
                const Candidate *tmp;
                if (t[0]->x > t[1]->x) { tmp = t[0]; t[0] = t[1]; t[1] = tmp; }
                if (t[1]->x > t[2]->x) { tmp = t[1]; t[1] = t[2]; t[2] = tmp; }
                if (t[0]->x > t[1]->x) { tmp = t[0]; t[0] = t[1]; t[1] = tmp; }

                /* Approximately horizontal alignment. */
                float ymin = fminf(t[0]->y, fminf(t[1]->y, t[2]->y));
                float ymax = fmaxf(t[0]->y, fmaxf(t[1]->y, t[2]->y));
                if (ymax - ymin > GEO_MAX_DY)
                    continue;

                /* Sufficient total width. */
                float width = t[2]->x - t[0]->x;
                if (width < GEO_MIN_WIDTH)
                    continue;

                /* Center dot roughly in the middle (1:1 spacing). */
                float d_left = t[1]->x - t[0]->x;
                float d_right = t[2]->x - t[1]->x;
                if (fabsf(d_left - d_right) > GEO_SYM_TOL * width)
                    continue;

                float score;
                if (has_valid_track) {
                    /* Gate against the last confirmed detection: the leader
                     * cannot teleport or change apparent size in one frame. */
                    float scale = width / track_width;
                    if (scale < TRACK_SCALE_MIN || scale > TRACK_SCALE_MAX)
                        continue;

                    float jump = hypotf(t[1]->x - track_cx, t[1]->y - track_cy);
                    if (jump > TRACK_MAX_JUMP_PX)
                        continue;
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

static BlobResult send_result(BlobResult *result) {
    if (dots_detection_queue &&
        xQueueOverwrite(dots_detection_queue, result) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send to queue");
    }
    return *result;
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

    /* ── Pass 4: EMA smoothing (paper: alpha = 0.8) ────────────────────── */

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
    has_valid_track = true;

    ESP_LOGD(TAG, "Triple: L(%.0f,%.0f) C(%.0f,%.0f) R(%.0f,%.0f) w=%.0f",
             result.dots[0].x, result.dots[0].y, result.dots[1].x,
             result.dots[1].y, result.dots[2].x, result.dots[2].y,
             result.spacing_px);

    return send_result(&result);
}

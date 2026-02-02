/**
 * @file dot_detection.c
 * @brief Fast red blob detection algorithm for ESP32-CAM
 *
 * Detects up to 3 red dots in YUV422 (YUYV) camera frames using
 * chrominance-based filtering and spatial clustering.
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "dot_detection.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "freertos/idf_additions.h"
#include "take_picture.h"

static const char *TAG = "blob_detect";

extern QueueHandle_t dots_detection_queue;

/* ─────────────────────────────────────────────────────────────────────────────
 * Configuration
 * ─────────────────────────────────────────────────────────────────────────── */

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

// Tracking persistence
#define MAX_BLIND_FRAMES 5 // Frames to keep last result when detection fails

/* ─────────────────────────────────────────────────────────────────────────────
 * Internal Types
 * ─────────────────────────────────────────────────────────────────────────── */

typedef struct
{
    long sum_x; // Sum of x coordinates (for centroid calculation)
    long sum_y; // Sum of y coordinates
    int count;  // Number of pixels in blob
} BlobAccumulator;

/* ─────────────────────────────────────────────────────────────────────────────
 * State
 * ─────────────────────────────────────────────────────────────────────────── */

static int blind_frame_count = 0;
static BlobResult last_good_result = {0};
static bool has_valid_track = false;

/* ─────────────────────────────────────────────────────────────────────────────
 * Helper Functions
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * Check if YUV pixel is red based on chrominance values.
 *
 * In YUV color space:
 *   - High V (Cr) indicates red/orange hue
 *   - Low U (Cb) excludes blue/purple tones
 *   - Y above threshold ensures sufficient brightness
 */
static inline bool is_red_pixel(uint8_t y, uint8_t u, uint8_t v)
{
    return (v >= RED_V_MIN) && (u < RED_U_MAX) && (y > LUMA_MIN);
}

/**
 * Find the nearest blob within clustering radius.
 * Returns blob index or -1 if no nearby blob exists.
 */
static int find_nearest_blob(BlobAccumulator *blobs, int count, int x, int y)
{
    int best_idx = -1;
    long best_dist = (long)CLUSTER_RADIUS * CLUSTER_RADIUS;

    for (int i = 0; i < count; i++)
    {
        if (blobs[i].count == 0)
            continue;

        int cx = blobs[i].sum_x / blobs[i].count;
        int cy = blobs[i].sum_y / blobs[i].count;
        int dx = x - cx;
        int dy = y - cy;

        // Early rejection for distant points
        if (abs(dx) > CLUSTER_RADIUS || abs(dy) > CLUSTER_RADIUS)
            continue;

        long dist_sq = (long)dx * dx + (long)dy * dy;
        if (dist_sq < best_dist)
        {
            best_dist = dist_sq;
            best_idx = i;
        }
    }
    return best_idx;
}

/**
 * Add pixel to existing blob or create new one.
 */
static void add_pixel_to_blobs(BlobAccumulator *blobs, int *active_count, int x, int y)
{
    int idx = find_nearest_blob(blobs, *active_count, x, y);

    if (idx >= 0)
    {
        // Add to existing blob
        blobs[idx].sum_x += x;
        blobs[idx].sum_y += y;
        blobs[idx].count += 1;
    }
    else if (*active_count < MAX_BLOBS)
    {
        // Create new blob
        idx = (*active_count)++;
        blobs[idx].sum_x = x;
        blobs[idx].sum_y = y;
        blobs[idx].count = 1;
    }
    // Else: too many blobs, pixel ignored
}

/**
 * Sort blobs by X coordinate (bubble sort - fine for small arrays).
 */
static void sort_blobs_by_x(struct Blob *blobs, int count)
{
    for (int i = 0; i < count - 1; i++)
    {
        for (int j = i + 1; j < count; j++)
        {
            if (blobs[j].cord_x < blobs[i].cord_x)
            {
                struct Blob tmp = blobs[i];
                blobs[i] = blobs[j];
                blobs[j] = tmp;
            }
        }
    }
}

/**
 * Send result to queue and return it.
 */
static BlobResult send_result(BlobResult *result, const char *msg)
{
    if (xQueueOverwrite(dots_detection_queue, result) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to send to queue");
    }
    if (msg)
    {
        ESP_LOGW(TAG, "%s", msg);
    }
    return *result;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Main Processing Function
 * ─────────────────────────────────────────────────────────────────────────── */

/**
 * Process camera frame and detect red blobs.
 *
 * @param fb Camera frame buffer (YUV422/YUYV format)
 * @return BlobResult with up to 3 detected blobs, coordinates centered on image
 *
 * YUV422 (YUYV) byte layout for 4 pixels:
 *   [Y0][U01][Y1][V01] [Y2][U23][Y3][V23] ...
 *
 * Each pair of pixels shares U and V values.
 */
BlobResult process_image(camera_fb_t *fb)
{
    BlobResult result = {0};

    // Validate input
    if (!fb || !fb->buf)
    {
        ESP_LOGE(TAG, "Invalid frame buffer");
        return send_result(&result, NULL);
    }

    const int width = fb->width;
    const int height = fb->height;
    const uint8_t *buf = fb->buf;
    const size_t buf_len = fb->len;

    // ─────────────────────────────────────────────────────────────────────────
    // Pass 1: Scan for red pixels and cluster into blobs
    // ─────────────────────────────────────────────────────────────────────────

    BlobAccumulator blobs[MAX_BLOBS] = {0};
    int active_blobs = 0;

    for (int y = 0; y < height; y += SCAN_STEP)
    {
        // YUV422: 2 bytes per pixel average (4 bytes per 2 pixels)
        int row_start = y * width * 2;

        for (int x = 0; x < width; x += SCAN_STEP)
        {
            // Calculate byte offset for this pixel pair
            // For pixel at x: Y is at x*2, U/V are shared between x and x+1
            int pair_base = row_start + (x & ~1) * 2; // Align to pixel pair

            if (pair_base + 4 > (int)buf_len)
                break;

            // YUYV layout: Y0, U, Y1, V
            uint8_t Y = buf[row_start + x * 2]; // Y for this pixel
            uint8_t U = buf[pair_base + 1];     // Shared U
            uint8_t V = buf[pair_base + 3];     // Shared V

            if (is_red_pixel(Y, U, V))
            {
                add_pixel_to_blobs(blobs, &active_blobs, x, y);
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Pass 2: Convert accumulators to blob candidates (filter by size)
    // ─────────────────────────────────────────────────────────────────────────

    struct Blob candidates[MAX_BLOBS];
    int candidate_count = 0;

    for (int i = 0; i < active_blobs; i++)
    {
        if (blobs[i].count >= MIN_BLOB_PIXELS)
        {
            candidates[candidate_count].cord_x = (float)blobs[i].sum_x / blobs[i].count;
            candidates[candidate_count].cord_y = (float)blobs[i].sum_y / blobs[i].count;
            candidates[candidate_count].count = blobs[i].count;
            candidate_count++;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Handle insufficient detections
    // ─────────────────────────────────────────────────────────────────────────

    if (candidate_count < 3)
    {
        // Use last known position for a few frames (smooths tracking)
        if (has_valid_track && blind_frame_count < MAX_BLIND_FRAMES)
        {
            blind_frame_count++;
            ESP_LOGW(TAG, "Blind frame %d/%d - using predicted position",
                     blind_frame_count, MAX_BLIND_FRAMES);
            return send_result(&last_good_result, NULL);
        }

        // Lost tracking completely
        blind_frame_count = 0;
        has_valid_track = false;
        memset(&result, 0, sizeof(result));

        char msg[64];
        snprintf(msg, sizeof(msg), "Detection failed: found %d blobs (need 3)", candidate_count);
        return send_result(&result, msg);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Pass 3: Sort and select top 3 blobs (left, center, right)
    // ─────────────────────────────────────────────────────────────────────────

    sort_blobs_by_x(candidates, candidate_count);

    // Take the 3 leftmost blobs
    for (int i = 0; i < 3; i++)
    {
        result.blobs[i] = candidates[i];
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Pass 4: Transform to image-centered coordinates
    // ─────────────────────────────────────────────────────────────────────────

    float center_x = width / 2.0f;
    float center_y = height / 2.0f;

    for (int i = 0; i < 3; i++)
    {
        result.blobs[i].cord_x -= center_x;                         // X: left negative, right positive
        result.blobs[i].cord_y = center_y - result.blobs[i].cord_y; // Y: up positive, down negative
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Update tracking state and output
    // ─────────────────────────────────────────────────────────────────────────

    blind_frame_count = 0;
    last_good_result = result;
    has_valid_track = true;

    ESP_LOGI(TAG, "Detected 3 blobs:");
    ESP_LOGI(TAG, "  LEFT:   (%.1f, %.1f) %d px",
             result.blobs[0].cord_x, result.blobs[0].cord_y, result.blobs[0].count);
    ESP_LOGI(TAG, "  CENTER: (%.1f, %.1f) %d px",
             result.blobs[1].cord_x, result.blobs[1].cord_y, result.blobs[1].count);
    ESP_LOGI(TAG, "  RIGHT:  (%.1f, %.1f) %d px",
             result.blobs[2].cord_x, result.blobs[2].cord_y, result.blobs[2].count);

    return send_result(&result, NULL);
}

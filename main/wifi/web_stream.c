
#include "dot_detection.h"
#include "web_log.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>

static const char *TAG = "web_stream";

/* Cap the debug stream frame rate. Every streamed frame costs a software
 * JPEG encode plus a camera framebuffer held for the whole send - an
 * unthrottled viewer starves the detection task of frames (FB-OVF). */
#define STREAM_FRAME_INTERVAL_MS 200

/* ── Detection overlay ────────────────────────────────────────────────────
 * Drawn straight into the YUYV frame before JPEG encoding: green boxes
 * around the three dots the detector currently believes are the LED triple,
 * plus a status square in the top-left corner (green = triple locked,
 * red = not found). Safe to modify the buffer here: this task owns it until
 * fb_return, and the DMA rewrites it before it is handed out again. */

static void yuyv_mark(uint8_t *buf, int width, int height, int x, int y,
                      uint8_t Y, uint8_t U, uint8_t V) {
    if (x < 0 || y < 0 || x >= width || y >= height)
        return;
    size_t px = ((size_t)y * width + x) * 2;
    size_t pair = px & ~(size_t)3; // [Y0 U Y1 V] group
    buf[px] = Y;
    buf[pair + 1] = U;
    buf[pair + 3] = V;
}

static void draw_box(uint8_t *buf, int w, int h, int cx, int cy, int half,
                     uint8_t Y, uint8_t U, uint8_t V) {
    for (int x = cx - half; x <= cx + half; x++) {
        yuyv_mark(buf, w, h, x, cy - half, Y, U, V);
        yuyv_mark(buf, w, h, x, cy + half, Y, U, V);
    }
    for (int y = cy - half; y <= cy + half; y++) {
        yuyv_mark(buf, w, h, cx - half, y, Y, U, V);
        yuyv_mark(buf, w, h, cx + half, y, Y, U, V);
    }
}

static void draw_detection_overlay(camera_fb_t *fb) {
    if (fb->format != PIXFORMAT_YUV422)
        return;

    BlobResult r = dot_detection_get_last();
    int w = fb->width, h = fb->height;

    /* Status square, top-left: green when locked, red when lost. */
    for (int y = 2; y < 12; y++)
        for (int x = 2; x < 12; x++)
            yuyv_mark(fb->buf, w, h, x, y, r.valid ? 170 : 80,
                      r.valid ? 0 : 90, r.valid ? 0 : 240);

    if (!r.valid)
        return;

    for (int i = 0; i < 3; i++) {
        /* Detector coords are image-centered; convert back to pixels. */
        int cx = (int)(r.dots[i].x + w / 2.0f);
        int cy = (int)(h / 2.0f - r.dots[i].y);
        /* Box size follows the blob's pixel count (scan grid is sparse). */
        int half = (int)(sqrtf((float)r.dots[i].count) * 2.0f) + 6;
        if (half > 40)
            half = 40;
        draw_box(fb->buf, w, h, cx, cy, half, 255, 0, 0); // bright green
    }
}

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
#define LEN_STREAM_BOUNDARY (sizeof("\r\n--" PART_BOUNDARY "\r\n") - 1)
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n";

/* Великий хендлер потоку.
 * user_ctx != NULL selects the /mask view: every pixel passing the red
 * thresholds is painted green, so the detector's vision can be tuned. */
esp_err_t jpg_stream_httpd_handler(httpd_req_t *req) {
    const bool paint_mask = (req->user_ctx != NULL);
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t jpg_buf_len = 0;
    uint8_t *jpg_buf = NULL;
    char part_buf[64];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    setsockopt(httpd_req_to_sockfd(req), SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        if (paint_mask)
            dot_detection_paint_mask(fb);
        draw_detection_overlay(fb);

        if (fb->format != PIXFORMAT_JPEG) {
            bool jpeg_converted = frame2jpg(fb, 80, &jpg_buf, &jpg_buf_len);
            if (!jpeg_converted) {
                ESP_LOGE(TAG, "JPEG compression failed");
                esp_camera_fb_return(fb);
                res = ESP_FAIL;
                break;
            }
        } else {
            jpg_buf_len = fb->len;
            jpg_buf = fb->buf;
        }

        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, LEN_STREAM_BOUNDARY);
        }
        if (res == ESP_OK) {
            int hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, jpg_buf_len);
            if (hlen > 0 && hlen < sizeof(part_buf)) {
                res = httpd_resp_send_chunk(req, part_buf, hlen);
            }
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_buf_len);
        }

        if (fb->format != PIXFORMAT_JPEG && jpg_buf) {
            free(jpg_buf);
            jpg_buf = NULL;
        }

        esp_camera_fb_return(fb);
        fb = NULL;

        if (res != ESP_OK) {
            ESP_LOGW(TAG, "Send failed, client disconnected");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(STREAM_FRAME_INTERVAL_MS));
    }

    return res;
}

static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t stream_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = jpg_stream_httpd_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &stream_uri);

        /* Threshold-mask debug view at /mask (any non-NULL ctx enables it) */
        static httpd_uri_t mask_uri;
        mask_uri = stream_uri;
        mask_uri.uri = "/mask";
        mask_uri.user_ctx = (void *)1;
        httpd_register_uri_handler(server, &mask_uri);

        /* Device log at /log (page) and /log.txt (raw) */
        web_log_register(server);
    }
    return server;
}

void web_stream_start(void) {
    web_log_init(); // start capturing logs for the /log page
    if (start_webserver() == NULL) {
        ESP_LOGE(TAG, "Failed to start web server");
    }
}
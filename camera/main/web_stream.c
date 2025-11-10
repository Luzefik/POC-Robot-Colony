
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "dots_algo.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>

static const char *TAG = "web_stream";

// Detection throttling
#ifndef DETECT_EVERY_N
#define DETECT_EVERY_N 20       // run detection every N frames
#endif
#ifndef DETECT_MAX_FRAME_MS
#define DETECT_MAX_FRAME_MS 200 // only run detection if previous frame time below this
#endif
#ifndef ENABLE_DETECTION
#define ENABLE_DETECTION 1     // 0=disabled for testing, 1=enabled
#endif

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n";


esp_err_t jpg_stream_httpd_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t jpg_buf_len = 0;
    uint8_t * jpg_buf = NULL;
    char part_buf[64];
    static int64_t last_frame = 0;
    static int frame_count = 0;
    static uint32_t prev_frame_time_ms = 0;
    static int detect_every_n = DETECT_EVERY_N; // run detection ~1-2 times per second
    static int detect_counter = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    // Set socket timeout to 5 seconds
    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    if (setsockopt(httpd_req_to_sockfd(req), SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0) {
        ESP_LOGE(TAG, "Failed to set socket timeout");
    }

    while(true){
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        if(fb->format != PIXFORMAT_JPEG){
            bool jpeg_converted = frame2jpg(fb, 80, &jpg_buf, &jpg_buf_len);
            if(!jpeg_converted){
                ESP_LOGE(TAG, "JPEG compression failed");
                esp_camera_fb_return(fb);
                res = ESP_FAIL;
                break;
            }
        } else {
            jpg_buf_len = fb->len;
            jpg_buf = fb->buf;
        }

        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(res == ESP_OK){
            int hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, jpg_buf_len);
            if(hlen < 0 || hlen >= sizeof(part_buf)){
                ESP_LOGE(TAG, "Header truncated");
                res = ESP_FAIL;
            } else {
                res = httpd_resp_send_chunk(req, part_buf, (size_t)hlen);
            }
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_buf_len);
        }

        // Lightweight gating: only attempt detection if previous frame was fast
        // and only every N frames to keep streaming smooth.
        if (ENABLE_DETECTION && res == ESP_OK && prev_frame_time_ms > 0 && prev_frame_time_ms < DETECT_MAX_FRAME_MS) {
            detect_counter++;
            if ((detect_counter % detect_every_n) == 0) {
                ESP_LOGD(TAG, "Triggering dots detection (prev_frame=%ums)", prev_frame_time_ms);
                // Prefer using fb to avoid extra copies; dots_algo decodes JPEG internally
                detect_dots(fb);
            }
        }

        if(fb->format != PIXFORMAT_JPEG){
            free(jpg_buf);
        }
        esp_camera_fb_return(fb);
        if(res != ESP_OK){
            ESP_LOGW(TAG, "Send failed, client disconnected or slow");
            break;
        }

    int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        float fps = frame_time > 0 ? 1000.0f / (float)frame_time : 0.0f;
    prev_frame_time_ms = (uint32_t)frame_time;

        frame_count++;
        if (frame_count % 50 == 0) {
            size_t free_heap = esp_get_free_heap_size();
            size_t min_heap = esp_get_minimum_free_heap_size();
            ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps) | Heap: free=%uKB min=%uKB",
                     (uint32_t)(jpg_buf_len/1024),
                     (uint32_t)frame_time, fps,
                     (uint32_t)(free_heap/1024),
                     (uint32_t)(min_heap/1024));
        } else {
            ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps)",
                     (uint32_t)(jpg_buf_len/1024),
                     (uint32_t)frame_time, fps);
        }

        // Skip frames if send is too slow (drop old frames from camera buffer)
        if (frame_time > 500) {
            ESP_LOGW(TAG, "Slow send detected (%ums), draining camera buffer", (uint32_t)frame_time);
            for (int i = 0; i < 3; i++) {
                camera_fb_t *drain_fb = esp_camera_fb_get();
                if (drain_fb) {
                    esp_camera_fb_return(drain_fb);
                }
            }
        }
    }    last_frame = 0;
    frame_count = 0;
    return res;
}

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {

        httpd_uri_t stream_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = jpg_stream_httpd_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &stream_uri);
    }
    return server;
}

void web_stream_start(void)
{
    if (start_webserver() == NULL) {
        ESP_LOGE(TAG, "Failed to start web server");
    }
}
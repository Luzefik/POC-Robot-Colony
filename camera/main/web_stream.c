// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

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

#ifndef DETECT_EVERY_N
#define DETECT_EVERY_N 20
#endif
#ifndef ENABLE_DETECTION
#define ENABLE_DETECTION 1
#endif

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
#define LEN_STREAM_BOUNDARY (sizeof("\r\n--" PART_BOUNDARY "\r\n") - 1)
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n";

esp_err_t jpg_stream_httpd_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t jpg_buf_len = 0;
    uint8_t *jpg_buf = NULL;
    char part_buf[64];
    static int64_t last_frame = 0;
    static int frame_count = 0;
    static int detect_every_n = DETECT_EVERY_N;
    static int detect_counter = 0;

    if (!last_frame) {
        last_frame = esp_timer_get_time();
    }

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

        if (ENABLE_DETECTION) {
            detect_counter++;
            if ((detect_counter % detect_every_n) == 0) {
                detect_dots(fb);
            }
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

        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        float fps = frame_time > 0 ? 1000.0f / (float)frame_time : 0.0f;

        frame_count++;
        if (frame_count % 30 == 0) {
            size_t free_heap = esp_get_free_heap_size();
            ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps) | Heap: %uKB",
                     (uint32_t)(jpg_buf_len / 1024),
                     (uint32_t)frame_time,
                     fps,
                     (uint32_t)(free_heap / 1024));
        }
    }

    last_frame = 0;
    frame_count = 0;
    return res;
}

static esp_err_t detection_handler(httpd_req_t *req) {
    detection_result_t *detection = get_detection_result();
    Point center = get_center_point();
    Point *dots = get_dots();
    int *sizes = get_dots_sizes();

    char response[512];
    int len = snprintf(response, sizeof(response),
        "{\"count\":%d,\"center\":{\"x\":%d,\"y\":%d},\"dots\":[",
        detection->count, center.x, center.y);

    for (int i = 0; i < detection->count; i++) {
        len += snprintf(response + len, sizeof(response) - len,
            "{\"id\":%d,\"x\":%d,\"y\":%d,\"size\":%d}%s",
            i, dots[i].x * 4, dots[i].y * 4, sizes[i],
            (i < detection->count - 1) ? "," : "");
    }

    snprintf(response + len, sizeof(response) - len, "]}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting server on port: %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t stream_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = jpg_stream_httpd_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &stream_uri);

        httpd_uri_t detection_uri = {
            .uri = "/api/detection",
            .method = HTTP_GET,
            .handler = detection_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &detection_uri);
    }
    return server;
}

void web_stream_start(void) {
    if (start_webserver() == NULL) {
        ESP_LOGE(TAG, "Failed to start web server");
    }
}
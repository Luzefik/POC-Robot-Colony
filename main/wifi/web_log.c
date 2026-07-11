#include "web_log.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Ring buffer for the most recent log output. 6 KB ~= the last few dozen
 * lines - enough to see why detection rejects a triple without scrolling. */
#define LOG_RING_SIZE 6144

static char s_ring[LOG_RING_SIZE];
static size_t s_head;
static bool s_wrapped;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t s_prev_vprintf;

/* Called by every ESP_LOGx in the system: format once, push into the ring
 * (minus ANSI color escapes), then forward to the previous sink (UART). */
static int log_tee_vprintf(const char *fmt, va_list args) {
    char line[256];
    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(line, sizeof(line), fmt, copy);
    va_end(copy);

    if (n > 0) {
        size_t len = (n < (int)sizeof(line)) ? (size_t)n : sizeof(line) - 1;

        taskENTER_CRITICAL(&s_mux);
        bool in_esc = false;
        for (size_t i = 0; i < len; i++) {
            char c = line[i];
            if (in_esc) {
                if (c == 'm')
                    in_esc = false;
                continue;
            }
            if (c == '\033') {
                in_esc = true;
                continue;
            }
            s_ring[s_head++] = c;
            if (s_head == LOG_RING_SIZE) {
                s_head = 0;
                s_wrapped = true;
            }
        }
        taskEXIT_CRITICAL(&s_mux);
    }

    return s_prev_vprintf ? s_prev_vprintf(fmt, args) : vprintf(fmt, args);
}

void web_log_init(void) {
    if (s_prev_vprintf == NULL)
        s_prev_vprintf = esp_log_set_vprintf(log_tee_vprintf);
}

static esp_err_t log_txt_handler(httpd_req_t *req) {
    char *out = malloc(LOG_RING_SIZE);
    if (!out)
        return httpd_resp_send_500(req);

    taskENTER_CRITICAL(&s_mux);
    size_t len;
    if (s_wrapped) {
        len = LOG_RING_SIZE;
        memcpy(out, s_ring + s_head, LOG_RING_SIZE - s_head);
        memcpy(out + LOG_RING_SIZE - s_head, s_ring, s_head);
    } else {
        len = s_head;
        memcpy(out, s_ring, s_head);
    }
    taskEXIT_CRITICAL(&s_mux);

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    esp_err_t res = httpd_resp_send(req, out, len);
    free(out);
    return res;
}

static esp_err_t log_page_handler(httpd_req_t *req) {
    static const char html[] =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<title>UGV log</title><style>"
        "body{background:#111;color:#ddd;font:13px/1.4 monospace;margin:0}"
        "pre{padding:12px;white-space:pre-wrap;word-break:break-all}"
        "</style></head><body><pre id='o'>loading...</pre><script>"
        "async function t(){try{const r=await fetch('/log.txt');"
        "document.getElementById('o').textContent=await r.text();"
        "window.scrollTo(0,document.body.scrollHeight);}catch(e){}}"
        "t();setInterval(t,1000);"
        "</script></body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

void web_log_register(httpd_handle_t server) {
    const httpd_uri_t log_txt = {
        .uri = "/log.txt", .method = HTTP_GET, .handler = log_txt_handler
    };
    const httpd_uri_t log_page = {
        .uri = "/log", .method = HTTP_GET, .handler = log_page_handler
    };
    httpd_register_uri_handler(server, &log_txt);
    httpd_register_uri_handler(server, &log_page);
}

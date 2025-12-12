// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#ifndef WEB_STREAM_H
#define WEB_STREAM_H

#include "esp_http_server.h"

esp_err_t jpg_stream_httpd_handler(httpd_req_t *req);

void web_stream_start(void);
#endif
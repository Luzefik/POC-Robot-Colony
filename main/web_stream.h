
#ifndef WEB_STREAM_H
#define WEB_STREAM_H

#include "esp_http_server.h"

esp_err_t jpg_stream_httpd_handler(httpd_req_t *req);

void web_stream_start(void);
#endif
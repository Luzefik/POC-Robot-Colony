#pragma once

#include "esp_http_server.h"

/* Tee ESP_LOG output into a small RAM ring buffer and serve it over HTTP:
 *   /log      - self-refreshing page (poll once a second)
 *   /log.txt  - raw text of the buffer
 * UART logging keeps working unchanged. */
void web_log_init(void);
void web_log_register(httpd_handle_t server);

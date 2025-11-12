#pragma once
#include "esp_err.h"

esp_err_t wifi_init_sta(void); 
typedef void (*wifi_got_ip_cb_t)(void);
void wifi_register_got_ip_cb(wifi_got_ip_cb_t cb);

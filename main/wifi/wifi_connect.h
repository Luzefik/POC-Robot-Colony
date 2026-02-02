// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#pragma once
#include "esp_err.h"

esp_err_t wifi_init_sta(void);
typedef void (*wifi_got_ip_cb_t)(void);
void wifi_register_got_ip_cb(wifi_got_ip_cb_t cb);

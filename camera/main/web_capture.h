#pragma once
#include "esp_err.h"

esp_err_t web_capture_start(void); // старт httpd, реєстрація /capture
esp_err_t web_capture_stop(void);  // зупинка сервера

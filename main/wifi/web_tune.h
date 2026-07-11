#pragma once

#include "esp_http_server.h"

/* Живий тюнінг детекції з браузера:
 *   /tune       - слайдери порогів, застосовуються миттєво (дивись /mask)
 *   /tune/get   - поточні значення (JSON)
 *   /tune/set   - змінити (GET-параметри, напр. /tune/set?red_v_min=140)
 *   /tune/save  - зберегти в NVS (переживає перезавантаження)
 *   /tune/reset - повернути дефолти з коду
 *   /probe?x=&y= - YUV значення пікселя (піпетка: клік по стріму на /) */
void web_tune_register(httpd_handle_t server);

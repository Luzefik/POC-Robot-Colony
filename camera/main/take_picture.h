#pragma once
#include "esp_err.h"

esp_err_t camera_init_board(void);      // ініціалізація камери
// опціонально: функції для зйомки
esp_err_t take_picture_once(void);      // робить 1 кадр і повертає (лог/файл)
void start_camera_task(void);           // якщо хочете фон. таск для постійної зйомки

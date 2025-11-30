#ifndef TAKE_PICTURE_H
#define TAKE_PICTURE_H

#include "esp_err.h"

esp_err_t camera_init_board(void);

esp_err_t take_picture_once(void);
void start_camera_task(void);

#endif

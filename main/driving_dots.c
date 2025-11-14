#include <driving.h>
#include <dots_algo.h>
#include "esp_log.h"

static const char *TAG = "MOV";

int16_t clamp(int x, int min, int max) {
    if (x < min) {
        return min;
    } else if (x >= max) {
        return max;
    }

    return x;
}

void mov(detection_data_t *data) {
    Point central_point = data->data[1];

    // Якщо немає даних - стоїмо
    if (data->count == 0) {
        motor(0, 0);
        motor(1, 0);
        motor(2, 0);
        motor(3, 0);
        return;
    }

    // Центр зображення (640x480 VGA)
    const int FRAME_CENTER_X = 320;

    // Обчислюємо відхилення від центру
    int error_x = central_point.x - FRAME_CENTER_X;

    ESP_LOGI(TAG, "count=%d, center_x=%d, error_x=%d",
             data->count, central_point.x, error_x);

    static int16_t turn = 0;
    int16_t speed = 150;
    int16_t tx = data->data[1].x;

    tx = tx / 10;


    if (turn < tx) {
        turn += 2;
    } else if (turn > tx) {
        turn -= 2;
    }

    if (speed > 0) {
        speed = abs(speed);
        motor(0, speed - (turn / 2));
        motor(1, 0);

        motor(2, speed + (turn / 2));
        motor(3, 0);
    }
}
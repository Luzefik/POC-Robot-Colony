#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_connect.h"
#include "esp_camera.h"
#include "new_algo.h"
#include "take_picture.h"
#include "web_stream.h"
#include "camera_pinout.h"
#include "freertos/task.h"
#include <cmath>
#include <driving.h>
#include <stdint.h>
#include "new_algo.h"
#include "math.h"


static const char *TAG = "app";

static void on_wifi_ready(void) {
    ESP_LOGI(TAG, "Wi-Fi ready → starting WEB STREAM");
    web_stream_start();
}


typedef struct {
    int16_t turn;
    int16_t acc;
} Conv;


static BlobResult dots = {0};

Conv getData() {
    int THRESH_HOLD_FOR_ACC  = 10;

    Conv result = {0,0};

    result.turn = (int16_t)dots.blobs[1].cord_x;


    int16_t gap_1_y = sqrt(pow(dots.blobs[0].cord_x - dots.blobs[1].cord_x, 2) + pow(dots.blobs[0].cord_y - dots.blobs[1].cord_y, 2));
    int16_t gap_2_y = sqrt(pow(dots.blobs[1].cord_x - dots.blobs[2].cord_x, 2) + pow(dots.blobs[1].cord_y - dots.blobs[2].cord_y, 2));

    int16_t gap_y = (gap_1_y + gap_2_y) / 2;

    result.acc = THRESH_HOLD_FOR_ACC - gap_y;

    return result;
}




static void detection_task(void *arg) {
    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        dots = process_image(fb);
        esp_camera_fb_return(fb);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(ret);

    if (camera_init_board() != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed");
        return;
    }

    ESP_LOGI(TAG, "Camera initialized");

    xTaskCreate(detection_task, "detection", 4096, NULL, 5, NULL);

    wifi_register_got_ip_cb(on_wifi_ready);
    wifi_init_sta();

    vTaskDelay(portMAX_DELAY);


    motor_init();
    for (;;) {
        Conv data = getData();
        int16_t x = data.turn;
        int16_t y = data.acc;

        static uint8_t turn = 0;
        static uint8_t speed = 0;

        if (speed > y) {speed -= 2;}
        else if (speed < y) {speed += 2;}

        if (turn > x) {turn -= 2;}
        else if (turn < x) {turn += 2;}

        motor(0, speed);
        motor(1, 0);
        motor(2, speed);
        motor(3, 0);

        if (speed > 0) {
            speed = abs(speed);
            motor(0, speed - (turn / 2));
            motor(1, 0);

            motor(2, speed + (turn / 2));
            motor(3, 0);
        } else {
            speed = abs(speed);
            motor(1, speed - (turn / 2));
            motor(0, 0);

            motor(3, speed + (turn / 2));
            motor(2, 0);
        }
    }
}




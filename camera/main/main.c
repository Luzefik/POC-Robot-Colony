#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_connect.h"
#include "esp_camera.h"
#include "new_algo.h"
#include "take_picture.h"
#include "web_stream.h"
#include "camera_pinout.h"
#include "freertos/task.h"
#include <driving.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "math.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"


static const char *TAG = "app";

static void on_wifi_ready(void) {
    ESP_LOGI(TAG, "Wi-Fi ready → starting WEB STREAM");
    web_stream_start();
}

QueueHandle_t dots_detection_queue;
typedef struct {
    int16_t turn;
    int16_t acc;
    bool valid;
} Conv;



Conv getData() {
    BlobResult dots = {0};
    // int THRESH_HOLD_FOR_ACC  = 10;
    Conv result = {0,0 ,false};
    if (xQueueReceive(dots_detection_queue, &dots, pdMS_TO_TICKS(100))) {
        ESP_LOGI(TAG, "Data received from queue");
        result.valid = true;
    } else {
        ESP_LOGW(TAG, "No data received from queue, using default values");
        result.valid = false;
        return result;
    }


    result.turn = (int16_t)dots.blobs[1].cord_x;


    int16_t gap_1_y = sqrt(pow(dots.blobs[0].cord_x - dots.blobs[1].cord_x, 2) + pow(dots.blobs[0].cord_y - dots.blobs[1].cord_y, 2));
    int16_t gap_2_y = sqrt(pow(dots.blobs[1].cord_x - dots.blobs[2].cord_x, 2) + pow(dots.blobs[1].cord_y - dots.blobs[2].cord_y, 2));

    ESP_LOGI(TAG, "LEFT DOT:   X=%.1f, Y=%.1f", dots.blobs[0].cord_x, dots.blobs[0].cord_y);
    ESP_LOGI(TAG, "CENTER DOT: X=%.1f, Y=%.1f", dots.blobs[1].cord_x, dots.blobs[1].cord_y);
    ESP_LOGI(TAG, "RIGHT DOT:  X=%.1f, Y=%.1f", dots.blobs[2].cord_x, dots.blobs[2].cord_y);


    int16_t gap_y = (gap_1_y + gap_2_y) / 2;

    ESP_LOGI(TAG, "gap_1_y: %.1f, gap_2_y: %.1f, gap_y %.1f", gap_1_y,gap_2_y,gap_y);

    result.acc =  gap_y;

    return result;
}




int16_t clamp(int x, int min, int max) {
    if (x < min) {
        return min;
    } else if (x >= max) {
        return max;
    }

    return x;
}


static void detection_task(void *arg) {
    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        // Process image and send to queue
        process_image(fb);
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

    dots_detection_queue = xQueueCreate(5, sizeof(BlobResult));
    if (dots_detection_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create dots_detection_queue");
        return;
    }
    ESP_LOGI(TAG, "Queue created successfully");


    motor_init();
    ESP_LOGI(TAG, "Motors initialized");

    xTaskCreate(detection_task, "detection", 8192, NULL, 5, NULL);

    // wifi_register_got_ip_cb(on_wifi_ready);
    // wifi_init_sta();



        for (;;) {
        static int16_t turn = 0;
        static int16_t speed = 0;

        // -512 <-> 512
        // 508 бо джойстик у нульовій позиціє для X та Y маюьть по 4 одиниці


        Conv data = getData();
        int16_t pacc = clamp(data.acc, -508, 508) / 4;
        int16_t px = clamp(data.turn, -508, 508) / 16;

        if (!data.valid) {
            if (turn >= 0) {
                motor(0, 350);
                motor(1, 0);
                motor(3, 0);
                motor(2, 0);
            } else {
                motor(0, 0);
                motor(1, 0);
                motor(2, 350);
                motor(3, 0);
            }

        } else {
            if (pacc > 0) {pacc += 351;}

            if (speed < pacc) {speed += 4;}
            if (speed > pacc) {speed -= 4;}

            if (0 < speed && speed < 351) {speed = 351;}
            px = clamp(px,-32, 31);
            if (px > turn) {turn += 2;}
            if (px < turn) {turn -= 2;}

            motor(0, speed + (turn ));
            motor(1, 0);
            motor(2, speed - (turn ));
            motor(3, 0);

        ESP_LOGI(TAG, "CONTROL - Turn: %d, Speed: %d, Acc: %d pacc: %d", turn, speed, data.acc, pacc);
        if (data.acc == 0) {
            speed = 0;
            ESP_LOGI(TAG, "Robot stopped");
        }
    }
    vTaskDelay( 50 / portTICK_PERIOD_MS);
}
}
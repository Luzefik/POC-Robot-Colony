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
#include <stdint.h>
#include "math.h"


#define MAX_PWM 1023
#define MIN_PWM 350  
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

BlobResult global_dots = {0};
SemaphoreHandle_t dots_mutex;
Conv getData() {
    int THRESH_HOLD_FOR_ACC  = 10;

    Conv result = {0,0};

    result.turn = (int16_t)global_dots.blobs[1].cord_x;


    int16_t gap_1_y = sqrt(pow(global_dots.blobs[0].cord_x - global_dots.blobs[1].cord_x, 2) + pow(global_dots.blobs[0].cord_y - global_dots.blobs[1].cord_y, 2));
    int16_t gap_2_y = sqrt(pow(global_dots.blobs[1].cord_x - global_dots.blobs[2].cord_x, 2) + pow(global_dots.blobs[1].cord_y - global_dots.blobs[2].cord_y, 2));
    int16_t gap_y = (gap_1_y + gap_2_y) / 2;

    ESP_LOGI(TAG, "gap_1_y: %.1f, gap_2_y: %.1f, gap_y %.1f", gap_1_y,gap_2_y,gap_y);


    ESP_LOGI(TAG, "LEFT DOT:   X=%.1f, Y=%.1f", global_dots.blobs[0].cord_x, global_dots.blobs[0].cord_y);
    ESP_LOGI(TAG, "CENTER DOT: X=%.1f, Y=%.1f", global_dots.blobs[1].cord_x, global_dots.blobs[1].cord_y);
    ESP_LOGI(TAG, "RIGHT DOT:  X=%.1f, Y=%.1f", global_dots.blobs[2].cord_x, global_dots.blobs[2].cord_y);

    result.acc = gap_y;

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
        if (!fb) { vTaskDelay(10); continue; }

        BlobResult local_res = process_image(fb);
        esp_camera_fb_return(fb);

        if (local_res.blobs[0].count > 0) {
            // Захищений запис
            xSemaphoreTake(dots_mutex, portMAX_DELAY);
            global_dots = local_res;
            xSemaphoreGive(dots_mutex);
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
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

    dots_mutex = xSemaphoreCreateMutex();

    xTaskCreate(detection_task, "detection", 8192, NULL, 5, NULL);

    // wifi_register_got_ip_cb(on_wifi_ready);
    // wifi_init_sta();

    motor_init();

    float speed = 0;
    float turn = 0;

    for (;;) {
        Conv data = getData();

        int target_speed = 0;

        if (data.acc > 20) {
            target_speed = (data.acc - 20) * 25;

            if (target_speed > MAX_PWM) target_speed = MAX_PWM;

            if (target_speed < MIN_PWM) target_speed = MIN_PWM;
        } else {
            target_speed = 0;
        }

        int target_turn = data.turn * 4;

        if (speed < target_speed) speed += 80;
        else if (speed > target_speed) speed -= 100;

        if (turn < target_turn) turn += 40;
        else if (turn > target_turn) turn -= 40;

        int left_pwm  = speed + turn - 10;
        int right_pwm = speed - turn;

        // 5. Обмеження (Clamp)
        if (left_pwm > MAX_PWM) left_pwm = MAX_PWM;
        if (left_pwm < -MAX_PWM) left_pwm = -MAX_PWM;

        if (right_pwm > MAX_PWM) right_pwm = MAX_PWM;
        if (right_pwm < -MAX_PWM) right_pwm = -MAX_PWM;

        if (left_pwm < 0) left_pwm = 0;
        if (right_pwm < 0) right_pwm = 0;

        motor(0, left_pwm);
        motor(1, 0);

        motor(2, right_pwm);
        motor(3, 0);

        ESP_LOGI(TAG, "Dist:%d -> L:%d R:%d", data.acc, left_pwm, right_pwm);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"


#include "driving.h"
#include "dots_algo.h"
#include "take_picture.h"
#include "btstack_port_esp32.h"
#include "btstack_run_loop.h"
#include "../src/components/bluepad32/include/uni.h"

static const char *TAG = "app";

// Global flag to track if gamepad is connected
// Flag set by Bluepad2 platform callbacks when the gamepad connects/disconnects.
// Mark volatile because it's written from the BT thread and read from the detection task.
static volatile bool gamepad_connected = false;

struct uni_platform* get_my_platform(void);

void set_gamepad_connected(bool connected) {
    gamepad_connected = connected;
}

static void init_nvs(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
}

static void init_motors(void) {
    motor_init();
    ESP_LOGI(TAG, "Motors initialized");
}

static void init_camera(void) {
    ESP_ERROR_CHECK(camera_init_board());
    ESP_LOGI(TAG, "Camera initialized");
}

static void detection_task(void *arg) {
    ESP_LOGI(TAG, "Detection task started - waiting for gamepad connection...");

    // Wait for gamepad to connect
    while (!gamepad_connected) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "Gamepad connected! Starting detection loop...");

    while (1) {
        // Stop detection if gamepad disconnects
        if (!gamepad_connected) {
            ESP_LOGI(TAG, "Gamepad disconnected - pausing detection");
            while (!gamepad_connected) {
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
            ESP_LOGI(TAG, "Gamepad reconnected - resuming detection");
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        detect_dots(fb);

        detection_result_t *detection = get_detection_result();
        Point center = get_center_point();

        if (detection->count > 0) {
            Point *dots = get_dots();
            int *sizes = get_dots_sizes();

            ESP_LOGI(TAG, "Dots: %d | Center: x=%d, y=%d | "
                     "P1:(%d,%d,%d) P2:(%d,%d,%d) P3:(%d,%d,%d)",
                     detection->count, center.x, center.y,
                     dots[0].x, dots[0].y, sizes[0],
                     dots[1].x, dots[1].y, sizes[1],
                     dots[2].x, dots[2].y, sizes[2]);
        }

        esp_camera_fb_return(fb);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

static void btstack_task(void *arg) {
    ESP_LOGI(TAG, "BTStack task started");

    // Blocking event loop - processes gamepad events
    btstack_run_loop_execute();

    vTaskDelete(NULL);
}


int app_main(void) {
    ESP_LOGI(TAG, "=== Robot ESP32 Starting ===");

    init_nvs();
    init_motors();
    init_camera();

    ESP_LOGI(TAG, "All systems initialized");

    ESP_LOGI(TAG, "Initializing BTStack...");
    btstack_init();
    ESP_LOGI(TAG, "BTStack initialized");

    ESP_LOGI(TAG, "Setting custom Bluepad32 platform...");
    uni_platform_set_custom(get_my_platform());
    ESP_LOGI(TAG, "Custom platform set");

    ESP_LOGI(TAG, "Initializing Bluepad32...");
    uni_init(0, NULL);
    ESP_LOGI(TAG, "Bluepad32 initialized");

    xTaskCreate(detection_task, "detection", 4096, NULL, 5, NULL);

    xTaskCreate(btstack_task, "btstack", 8192, NULL, 10, NULL);

    ESP_LOGI(TAG, "All tasks created and running");

    vTaskDelay(portMAX_DELAY);

    return 0;
}
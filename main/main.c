#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "driving.h"
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
    ESP_LOGI(TAG, "Motors initialized");

    ESP_LOGI(TAG, "Initializing BTStack...");
    btstack_init();
    ESP_LOGI(TAG, "BTStack initialized");

    ESP_LOGI(TAG, "Setting custom Bluepad32 platform...");
    uni_platform_set_custom(get_my_platform());
    ESP_LOGI(TAG, "Custom platform set");

    ESP_LOGI(TAG, "Initializing Bluepad32...");
    uni_init(0, NULL);
    ESP_LOGI(TAG, "Bluepad32 initialized");

    ESP_LOGI(TAG, "Starting BT run loop...");
    btstack_run_loop_execute();

    return 0;
}
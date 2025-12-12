#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_connect.h"
#include "esp_camera.h"
#include "dots_algo.h"
#include "web_stream.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include "driving.h"
#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include <hci_dump.h>
#include <hci_dump_embedded_stdout.h>
#include <uni.h>

#include "sdkconfig.h"

// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

#define LEADER 1

static const char *TAG = "app";


QueueHandle_t camera_to_motor_queue;

#ifdef LEADER
struct uni_platform* get_my_platform(void);

int app_main(void) {
    motor_init();
    // If you enable HCI Dump better to disable "Bluepad32 USB Console" from "idf.py menuconfig".
    // hci_dump_init(hci_dump_embedded_stdout_get_instance());

    // Don't use BTstack buffered UART. It conflicts with the console.
#ifdef CONFIG_ESP_CONSOLE_UART
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
    btstack_stdio_init();
#endif  // CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
#endif  // CONFIG_ESP_CONSOLE_UART

    // Configure BTstack for ESP32 VHCI Controller
    btstack_init();

    // Must be called before uni_init()
    uni_platform_set_custom(get_my_platform());

    // Init Bluepad32.
    uni_init(0 /* argc */, NULL /* argv */);

    // Does not return.
    btstack_run_loop_execute();

    return 0;
}
#else

esp_err_t camera_init_board(void);

static void on_wifi_ready(void) {
    ESP_LOGI(TAG, "Wi-Fi ready → starting WEB STREAM");
    web_stream_start();
}

static void camera_detection_task(void *arg) {
    int frame_count = 0;
    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Failed to get camera frame!");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        frame_count++;
        ESP_LOGI(TAG, "Frame %d: format=%d, len=%d, w=%d, h=%d",
                 frame_count, fb->format, fb->len, fb->width, fb->height);

        detect_dots(fb);
        detection_data_t dots_data = get_detection_data();

        ESP_LOGI(TAG, "Detection result: count=%d", dots_data.count);

        xQueueSend(camera_to_motor_queue, &dots_data, pdMS_TO_TICKS(10));

        esp_camera_fb_return(fb);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

static void motor_control_task(void *arg) {
    while (1)
    {
        detection_data_t latest_dot_data;
        xQueueReceive(camera_to_motor_queue, &latest_dot_data, pdMS_TO_TICKS(10));

        mov(&latest_dot_data);
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

    motor_init();
    ESP_LOGI(TAG, "Motors initialized");

    camera_to_motor_queue = xQueueCreate(3, sizeof(detection_data_t));

    xTaskCreate(camera_detection_task, "detection", 4096, NULL, 5, NULL);
    xTaskCreate(motor_control_task, "motor_brain", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "Всі завдання запущені. app_main завершує роботу.");

    // wifi_register_got_ip_cb(on_wifi_ready);
    // wifi_init_sta();

    // vTaskDelay(portMAX_DELAY);
}
#endif
// #else  // НЕ-ЛІДЕР --- IGNORE ---
// esp_err_t camera_init_board(void);

// static void on_wifi_ready(void) {
//     ESP_LOGI(TAG, "Wi-Fi ready → starting WEB STREAM");
//     web_stream_start();
// }

// static void detection_task(void *arg) {
//     while (1) {
//         camera_fb_t *fb = esp_camera_fb_get();
//         if (!fb) {
//             vTaskDelay(100 / portTICK_PERIOD_MS);
//             continue;
//         }

//         detect_dots(fb);
//         esp_camera_fb_return(fb);
//         vTaskDelay(50 / portTICK_PERIOD_MS);
//     }
// }

// void app_main(void) {
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ESP_ERROR_CHECK(nvs_flash_init());
//     }
//     ESP_ERROR_CHECK(ret);

//     if (camera_init_board() != ESP_OK) {
//         ESP_LOGE(TAG, "Camera init failed");
//         return;
//     }

//     ESP_LOGI(TAG, "Camera initialized");

//     xTaskCreate(detection_task, "detection", 4096, NULL, 5, NULL);

//     wifi_register_got_ip_cb(on_wifi_ready);
//     wifi_init_sta();

//     vTaskDelay(portMAX_DELAY);
// }
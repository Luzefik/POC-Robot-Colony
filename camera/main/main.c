#include "esp_log.h"
#include "nvs_flash.h"

#include "wifi_connect.h"
#include "take_picture.h"
#include "web_capture.h"

#include "esp_psram.h"

static const char *TAG = "app";

static void on_wifi_ready(void)
{
    // wifi_connect викликав цю функцію коли отримали IP
    ESP_LOGI(TAG, "Wi-Fi up → start web server");
    web_capture_start();   // реєструє /capture і починає слухати
}

void app_main(void)
{
    // 1) NVS (для Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(ret);


    // size_t psram_size = esp_psram_get_size();
    // ESP_LOGE(TAG, "PSRAM size: %u bytes\n", psram_size);
    // printf("PSRAM size: %u bytes\n", psram_size);

    // 2) Ініціалізуємо камеру
    if (camera_init_board() != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed");
        return;
    }

    // 3) реєструємо колбек отримання IP і запускаємо Wi-Fi
    wifi_register_got_ip_cb(on_wifi_ready);
    wifi_init_sta(); // після підключення викличе on_wifi_ready()

    // 4) (опційно) старт таску камери / періодичної зйомки
    // start_camera_task();
}

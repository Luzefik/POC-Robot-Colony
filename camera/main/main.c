#include "esp_log.h"
#include "nvs_flash.h"

#include "wifi_connect.h"
#include "take_picture.h"
#include "web_capture.h"

#include "esp_psram.h"

static const char *TAG = "app";

static void on_wifi_ready(void)
{
    ESP_LOGI(TAG, "Wi-Fi up → start web server");
    web_capture_start();
}

void app_main(void)
{
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

    wifi_register_got_ip_cb(on_wifi_ready);
    wifi_init_sta(); 

}

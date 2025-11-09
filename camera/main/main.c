#include "esp_log.h"
#include "nvs_flash.h"

#include "wifi_connect.h"
#include "take_picture.h"


// #include "web_capture.h"
#include "web_stream.h"


#include "esp_psram.h"

static const char *TAG = "app";

#define APP_MODE_STREAM 1

/*
APP_MODE_STREAM 1 - stream
APP_MODE_STREAM 0 - capture
*/
#if defined(APP_MODE_STREAM)
    #include "web_stream.h"
#else
    #include "web_capture.h"
#endif

static void on_wifi_ready(void)
{
#if defined(APP_MODE_STREAM)
    ESP_LOGI(TAG, "Wi-Fi up → start WEB STREAM server");
    web_stream_start();
#else
    ESP_LOGI(TAG, "Wi-Fi up → start WEB CAPTURE server");
    web_capture_start();
#endif
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

idf_component_register(
    SRCS "main.c" "wifi_connect.c" "take_picture.c" "web_capture.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_wifi esp_event esp_netif esp_http_server nvs_flash esp_timer esp_psram)

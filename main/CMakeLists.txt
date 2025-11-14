set(srcs
    "main.c"
    "my_platform.c"
    "driving.cpp"
    "take_picture.c"
    "dots_algo.c"
    "web_stream.c"
    "wifi_connect.c"
)


set(requires
    "bluepad32"
    "btstack"
)

set(priv_requires
    "esp_timer"
    "esp32-camera"
    "esp_jpeg"
    "esp_driver_gpio"
    "esp_driver_ledc"
    "lwip"
    "esp_http_server"
    "esp_wifi"
    "esp_event"
    "esp_netif"
)

idf_component_register(
    SRCS "driving_dots.c" "${srcs}"
    INCLUDE_DIRS "."
    REQUIRES "${requires}"
    PRIV_REQUIRES "${priv_requires}"
)

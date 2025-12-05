idf_component_register(
    SRCS
        "main.c"
        "i2c_lcd.c"
        "driving.cpp"
        "new_algo.c"
        "dots_algo.c"
        "take_picture.c"
        "web_stream.c"
        "web_capture.c"

    INCLUDE_DIRS
        "."

    REQUIRES
        esp_http_server
        nvs_flash
        esp_timer
        driver
        esp_driver_ledc
        espressif__esp32-camera
        espressif__esp_jpeg
)
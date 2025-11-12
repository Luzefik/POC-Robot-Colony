set(srcs
    "main.c"
    "my_platform.c"
    "driving.cpp"
    "take_picture.c"
    "dots_algo.c"
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
)

idf_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS "."
    REQUIRES "${requires}"
    PRIV_REQUIRES "${priv_requires}"
)

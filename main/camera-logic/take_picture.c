#include "take_picture.h"
#include "camera_pinout.h"

#include "esp_camera.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "camera";

static const camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 20000000,
    /* XCLK is generated on the LOW_SPEED LEDC unit inside the camera
     * driver; the motor PWM uses the HIGH_SPEED unit (driving.cpp). */
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    /* YUV422 (YUYV): the detection thresholds work directly on the U/V
     * chroma channels ("Column of Ground Robots", V.B). CIF keeps the
     * processing loop fast enough for the 20 Hz control cycle. */
    .pixel_format = PIXFORMAT_YUV422,
    .frame_size = FRAMESIZE_CIF,

    .fb_count = 2,
    .fb_location = CAMERA_FB_IN_PSRAM,
    /* Always hand the freshest frame to the detector: stale frames add
     * dead time to the control loop and cause target loss. */
    .grab_mode = CAMERA_GRAB_LATEST,
};

esp_err_t camera_init_board(void) {
    /* Sensor init sometimes NACKs mid-way on a marginal power rail or after
     * a warm reset; a deinit + retry recovers it in practice. */
    esp_err_t err = ESP_FAIL;
    for (int attempt = 1; attempt <= 3; attempt++) {
        err = esp_camera_init(&camera_config);
        if (err == ESP_OK)
            return ESP_OK;

        ESP_LOGW(TAG, "Camera init attempt %d/3 failed: %s", attempt,
                 esp_err_to_name(err));
        esp_camera_deinit();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
    return err;
}

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
    /* XCLK камера генерує на LOW_SPEED-блоці LEDC (всередині драйвера);
     * PWM моторів сидить на HIGH_SPEED-блоці (driving.cpp), тому вони
     * не конфліктують, хоч номери таймера/каналу однакові. */
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    /* YUV422 (YUYV): пороги детекції працюють прямо з каналами U/V без
     * конвертації кольору. CIF (~400x296) - баланс між точністю і
     * швидкістю: сенсор видає ~11 кадрів/с. */
    .pixel_format = PIXFORMAT_YUV422,
    .frame_size = FRAMESIZE_CIF,

    /* 3 буфери: один тримає детекція, другий може тримати стрім (поки
     * кодує JPEG), а DMA камери потрібен третій вільний. З двома буферами
     * драйвер губить кадри (FB-OVF), щойно хтось відкриває стрім. */
    .fb_count = 3,
    .fb_location = CAMERA_FB_IN_PSRAM,
    /* Детектору завжди даємо найсвіжіший кадр: застарілі кадри - це
     * мертвий час у контурі керування і втрата цілі. */
    .grab_mode = CAMERA_GRAB_LATEST,
};

/* Камера на шасі змонтована догори ногами: повертаємо картинку на 180
 * градусів на рівні сенсора (vflip + hmirror). Так і стрім, і координати
 * для керування стають правильними. Якщо модуль стоїть нормально -
 * постав 0. */
#define CAMERA_ROTATE_180 1

esp_err_t camera_init_board(void) {
    /* Ініціалізація сенсора інколи обривається по I2C (слабке живлення,
     * теплий рестарт, завислий сусід на шині) - тому до трьох спроб
     * з паузою. Якщо не завелось і так - перевіряй залізо: шлейф камери,
     * живлення, іншу периферію на пінах 26/27. */
    esp_err_t err = ESP_FAIL;
    for (int attempt = 1; attempt <= 3; attempt++) {
        err = esp_camera_init(&camera_config);
        if (err == ESP_OK) {
#if CAMERA_ROTATE_180
            sensor_t *s = esp_camera_sensor_get();
            if (s) {
                s->set_vflip(s, 1);
                s->set_hmirror(s, 1);
            }
#endif
            return ESP_OK;
        }

        ESP_LOGW(TAG, "Camera init attempt %d/3 failed: %s", attempt,
                 esp_err_to_name(err));
        esp_camera_deinit();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
    return err;
}

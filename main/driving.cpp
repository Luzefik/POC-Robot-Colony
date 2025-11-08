#include "driving.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define MOTOR_PWM_FREQ_HZ       5000
#define MOTOR_PWM_RESOLUTION    LEDC_TIMER_8_BIT
#define MOTOR_MAX_DUTY          ((1 << 8) - 1)
#define TAG "MOTOR"

static const int motor_gpio_pins[4] = { 4, 5, 18, 19 };

static const ledc_channel_t motor_channels[4] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3
};

#define MOTOR_LEDC_TIMER LEDC_TIMER_0
#define MOTOR_SPEED_MODE LEDC_HIGH_SPEED_MODE

void motor_init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = MOTOR_SPEED_MODE,
        .duty_resolution  = MOTOR_PWM_RESOLUTION,
        .timer_num        = MOTOR_LEDC_TIMER,
        .freq_hz          = MOTOR_PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = MOTOR_SPEED_MODE,
            .channel        = motor_channels[i],
            .timer_sel      = MOTOR_LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = motor_gpio_pins[i],
            .duty           = 0,
            .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }
}

void motor_set_speed(int motor_id, float speed_percent) {
    if (speed_percent < 0.0f) speed_percent = 0.0f;
    if (speed_percent > 100.0f) speed_percent = 100.0f;

    uint32_t duty = (uint32_t)((speed_percent / 100.0f) * MOTOR_MAX_DUTY);
    ESP_ERROR_CHECK(ledc_set_duty(MOTOR_SPEED_MODE, motor_channels[motor_id], duty));
    ESP_ERROR_CHECK(ledc_update_duty(MOTOR_SPEED_MODE, motor_channels[motor_id]));
}

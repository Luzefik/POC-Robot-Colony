#include "driving.h"
#include "driver/ledc.h"

#define MOTOR_PWM_FREQ_HZ 5000
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_10_BIT
#define MOTOR_MAX_DUTY 1023
#define TAG "MOTOR"
#define MAX_PWM 1023
#define MIN_PWM 350  // Мінімальна сила, щоб колеса крутились (підбирай: 300-400)

static const int motor_gpio_pins[4] = {12, 13, 14, 15};

static const ledc_channel_t motor_channels[4] = {
    LEDC_CHANNEL_4, LEDC_CHANNEL_5, LEDC_CHANNEL_6, LEDC_CHANNEL_7
};

#define MOTOR_LEDC_TIMER LEDC_TIMER_1
#define MOTOR_SPEED_MODE LEDC_LOW_SPEED_MODE

void motor_init(void) {
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = MOTOR_SPEED_MODE;
    ledc_timer.timer_num = MOTOR_LEDC_TIMER;
    ledc_timer.duty_resolution = MOTOR_PWM_RESOLUTION;
    ledc_timer.freq_hz = MOTOR_PWM_FREQ_HZ;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer.deconfigure = false;

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ledc_channel = {};
        ledc_channel.gpio_num = motor_gpio_pins[i];
        ledc_channel.speed_mode = MOTOR_SPEED_MODE;
        ledc_channel.channel = motor_channels[i];
        ledc_channel.timer_sel = MOTOR_LEDC_TIMER;
        ledc_channel.duty = 0;
        ledc_channel.hpoint = 0;
        ledc_channel.flags.output_invert = 0;

        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }
}

void motor(int motor_id, int pwm) {
    ledc_set_duty(MOTOR_SPEED_MODE, motor_channels[motor_id], pwm);
    ledc_update_duty(MOTOR_SPEED_MODE, motor_channels[motor_id]);
}

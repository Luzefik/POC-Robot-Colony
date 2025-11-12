#include "driving.h"
#include "driver/ledc.h"

#define MOTOR_PWM_FREQ_HZ 5000
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_8_BIT
#define MOTOR_MAX_DUTY 255
#define TAG "MOTOR"

static const int motor_gpio_pins[4] = {4, 5, 18, 19};

static const ledc_channel_t motor_channels[4] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3
};

#define MOTOR_LEDC_TIMER LEDC_TIMER_0
#define MOTOR_SPEED_MODE LEDC_HIGH_SPEED_MODE

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

<<<<<<< HEAD
void motor(int motor_id, int pwm) {
    ledc_set_duty(MOTOR_SPEED_MODE, motor_channels[motor_id], pwm);
    ledc_update_duty(MOTOR_SPEED_MODE, motor_channels[motor_id]);
=======
void motor_set_speed(int motor_id,
                     float speed_percent) {
    if (speed_percent < 0.0f) speed_percent = 0.0f;
    if (speed_percent > 100.0f) speed_percent = 100.0f;


    if (motor_id == 0 ) {
        ledc_set_duty(MOTOR_SPEED_MODE, motor_channels[1], 0)
        ledc_update_duty(MOTOR_SPEED_MODE, motor_channels[0])
    }
    if (motor_id == 1) {
        ledc_set_duty(MOTOR_SPEED_MODE, motor_channels[0], 0)
                ledc_update_duty(MOTOR_SPEED_MODE, motor_channels[1])

    }

    if (motor_id == 2) {
        ledc_set_duty(MOTOR_SPEED_MODE, motor_channels[3], 0)
                ledc_update_duty(MOTOR_SPEED_MODE, motor_channels[2])

    }


    if (motor_id == 3) {
        ledc_set_duty(MOTOR_SPEED_MODE, motor_channels[2], 0)
                ledc_update_duty(MOTOR_SPEED_MODE, motor_channels[3])

    }

    uint32_t duty = (uint32_t)((speed_percent / 100.0f) * MOTOR_MAX_DUTY);
    ESP_ERROR_CHECK(ledc_set_duty(MOTOR_SPEED_MODE, motor_channels[motor_id], duty));
    ESP_ERROR_CHECK(ledc_update_duty(MOTOR_SPEED_MODE, motor_channels[motor_id]));

>>>>>>> ab6cc72a597933a6354417564e72cb6ec41b97a2
}


// void motor_set_speed(int x, int y
//                     int motor_id_1, int motor_id_2,
//                     float speed_percent_1, float speed_percent_2) {
//     if (speed_percent_1 < 0.0f) speed_percent_1 = 0.0f;
//     if (speed_percent_2 < 0.0f) speed_percent_2 = 0.0f;

//     if (speed_percent_1 > 100.0f) speed_percent_1 = 100.0f;
//     if (speed_percent_2 > 100.0f) speed_percent_2 = 100.0f;

//     if y>0 {
//         uint32_t duty_1 = (uint32_t)((speed_percent_1 / 100.0f) * MOTOR_MAX_DUTY);
//         uint32_t duty_2 = (uint32_t)((speed_percent_2 / 100.0f) * MOTOR_MAX_DUTY);
//         ESP_ERROR_CHECK(ledc_set_duty(MOTOR_SPEED_MODE, motor_channels[motor_id_1], duty1));
//         ESP_ERROR_CHECK(ledc_set_duty(MOTOR_SPEED_MODE, motor_channels[motor_id_2], duty2));
//     }



}
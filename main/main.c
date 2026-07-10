// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java:
// https://pvs-studio.com

#include "dot_detection.h"
#include "driving.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "take_picture.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#if CONFIG_UGV_ENABLE_WEB_STREAM
#include "web_stream.h"
#include "wifi_connect.h"
#endif

#if CONFIG_BT_ENABLED
#include "btstack_port_esp32.h"
#include "btstack_run_loop.h"
#include "uni.h"
struct uni_platform *get_my_platform(void); // driving-logic/my_platform.c
#endif

#if CONFIG_UGV_MODE_SWITCH_GPIO >= 0
#include "driver/gpio.h"
#endif

static const char *TAG = "app";

/* ── Role selection ──────────────────────────────────────────────────────
 * When no SW1 switch is configured (CONFIG_UGV_MODE_SWITCH_GPIO = -1),
 * the robot boots into this role. Flip the value and rebuild:
 *   0 = FOLLOWER (vision tracking), 1 = LEADER (Bluetooth gamepad).
 * With a switch configured, the GPIO level decides instead (1 = leader). */
#define DEFAULT_ROLE_LEADER 0

QueueHandle_t dots_detection_queue;

/* ── Follower control ("Column of Ground Robots", section III.B) ─────────
 *
 * Three-stage pipeline at 20 Hz: signal preprocessing (eq. 1-2), velocity
 * smoothing with dead-zone compensation (eq. 3), and motor mixing (eq. 4).
 */
#define CONTROL_PERIOD_MS 50 // 20 Hz control loop

#define TURN_DIV 16 // eq. 1: E_turn = x_error / 16
#define VEL_DIV 4   // eq. 2: E_vel = spacing_error / 4

#define RAMP_STEP_V 4 // eq. 3: max velocity change per cycle, PWM units
#define RAMP_STEP_W 2

#define PWM_DEADZONE 351   // static friction compensation of the gearboxes
#define MIN_ACTIVE_CMD 3   // commands below this are treated as zero
#define MAX_V_CMD 120      // clamp for the ramped commands, PWM units
#define MAX_W_CMD 80

/* Apparent width of the LED triple at the desired following distance.
 * Larger spacing = leader closer (spacing ~ 1/distance). Tune on hardware. */
#define TARGET_SPACING_PX 70
#define SPACING_DEADBAND_PX 8 // hold position inside this error band

#define SEARCH_AFTER_MS 100 // paper: rotate in place after 100 ms without data
#define SEARCH_PWM 350
#define LOST_STOP_MS 3000 // fail-safe: full stop when the leader is gone

/* ── Gamepad state (referenced by driving-logic/my_platform.c) ─────────── */

static volatile bool s_gamepad_connected;

void set_gamepad_connected(bool connected) {
    s_gamepad_connected = connected;
    ESP_LOGI(TAG, "Gamepad %s", connected ? "connected" : "disconnected");
    if (!connected)
        drive_stop(); // fail-safe: never keep driving without a pilot
}

/* ── Detection task ────────────────────────────────────────────────────── */

static void detection_task(void *arg) {
    int frames = 0;
    int64_t window_start = esp_timer_get_time();

    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Frame grab failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        process_image(fb);
        esp_camera_fb_return(fb);

        /* Compact once-a-second stats instead of per-frame log flood. */
        frames++;
        int64_t now = esp_timer_get_time();
        if (now - window_start >= 1000000) {
            ESP_LOGI(TAG, "Detection: %.1f fps", frames * 1000000.0f / (now - window_start));
            frames = 0;
            window_start = now;
        }

        vTaskDelay(1);
    }
}

/* ── Follower mode ─────────────────────────────────────────────────────── */

static int slew(int cur, int target, int step) {
    if (cur < target)
        return (cur + step > target) ? target : cur + step;
    if (cur > target)
        return (cur - step < target) ? target : cur - step;
    return cur;
}

/* Dead-zone compensation (eq. 3): any non-zero command gets PWM_DEADZONE
 * added to overcome the static friction of the DC gearboxes. */
static int apply_deadzone(int cmd) {
    if (abs(cmd) < MIN_ACTIVE_CMD)
        return 0;
    int mag = clampi(abs(cmd), 0, MOTOR_MAX_DUTY - PWM_DEADZONE);
    return (cmd > 0) ? PWM_DEADZONE + mag : -(PWM_DEADZONE + mag);
}

static void follower_loop(void) {
    int v = 0, w = 0;          // ramped velocity / turn commands
    float last_heading = 0.0f; // last known direction to the leader, px
    int64_t last_valid_us = esp_timer_get_time();
    bool stopped_logged = false;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        BlobResult r;
        bool valid = xQueueReceive(dots_detection_queue, &r,
                                   pdMS_TO_TICKS(CONTROL_PERIOD_MS)) == pdTRUE &&
                     r.valid;

        if (valid) {
            last_valid_us = esp_timer_get_time();
            stopped_logged = false;

            /* eq. 1: heading error from the center dot (already frame-centered) */
            float heading_err = r.dots[1].x;
            last_heading = heading_err;

            /* eq. 2: distance error from the triple width */
            float spacing_err = TARGET_SPACING_PX - r.spacing_px; // >0: too far

            int v_target = 0;
            if (fabsf(spacing_err) > SPACING_DEADBAND_PX)
                v_target = clampi((int)(spacing_err / VEL_DIV), -MAX_V_CMD, MAX_V_CMD);
            int w_target = clampi((int)(heading_err / TURN_DIV), -MAX_W_CMD, MAX_W_CMD);

            /* eq. 3: incremental update, no instantaneous state changes */
            v = slew(v, v_target, RAMP_STEP_V);
            w = slew(w, w_target, RAMP_STEP_W);

            /* eq. 4: motor mixing */
            drive_left(apply_deadzone(v + w));
            drive_right(apply_deadzone(v - w));

            ESP_LOGD(TAG, "x_err=%.0f spacing=%.0f v=%d w=%d", heading_err,
                     r.spacing_px, v, w);
        } else {
            int64_t lost_ms = (esp_timer_get_time() - last_valid_us) / 1000;
            v = 0;
            w = 0;

            if (lost_ms > LOST_STOP_MS) {
                /* Fail-safe: target is gone, stop instead of spinning forever
                 * (with several robots losing the leader at once, endless
                 * search mode destroys the column - see paper VI.B). */
                drive_stop();
                if (!stopped_logged) {
                    ESP_LOGW(TAG, "Leader lost for %lld ms - stopping", lost_ms);
                    stopped_logged = true;
                }
            } else if (lost_ms > SEARCH_AFTER_MS) {
                /* Search mode: rotate in place toward the last known vector. */
                int dir = (last_heading >= 0) ? 1 : -1;
                drive_left(dir * SEARCH_PWM);
                drive_right(-dir * SEARCH_PWM);
            }
            /* else: brief dropout, keep the last motor command */
        }

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

static void follower_main(void) {
    ESP_LOGI(TAG, "Starting in FOLLOWER mode (vision tracking)");

    if (camera_init_board() != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed");
        return;
    }

    dots_detection_queue = xQueueCreate(1, sizeof(BlobResult));
    if (dots_detection_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create dots_detection_queue");
        return;
    }

    motor_init();

#if CONFIG_UGV_ENABLE_WEB_STREAM
    wifi_register_got_ip_cb(web_stream_start);
    wifi_init_sta();
#endif

    /* Vision on core 1, control loop stays on core 0 with the main task. */
    xTaskCreatePinnedToCore(detection_task, "detection", 8192, NULL, 5, NULL, 1);

    follower_loop();
}

/* ── Leader mode ───────────────────────────────────────────────────────── */

static void leader_main(void) {
#if CONFIG_BT_ENABLED
    ESP_LOGI(TAG, "Starting in LEADER mode (Bluetooth gamepad)");

    motor_init();

    btstack_init();
    uni_platform_set_custom(get_my_platform());
    uni_init(0, NULL);

    btstack_run_loop_execute(); // does not return
#else
    ESP_LOGE(TAG, "Leader mode requires CONFIG_BT_ENABLED "
                  "(see sdkconfig.defaults); staying idle");
#endif
}

/* ── Entry point ───────────────────────────────────────────────────────── */

/* SW1 selects leader/follower at boot without reflashing (paper IV.B).
 * Configure the GPIO via menuconfig: "UGV column configuration". */
static bool mode_is_leader(void) {
#if CONFIG_UGV_MODE_SWITCH_GPIO >= 0
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << CONFIG_UGV_MODE_SWITCH_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    vTaskDelay(pdMS_TO_TICKS(10));
    return gpio_get_level(CONFIG_UGV_MODE_SWITCH_GPIO) == 1;
#else
    return DEFAULT_ROLE_LEADER != 0;
#endif
}

void app_main(void) {
    if (mode_is_leader())
        leader_main();
    else
        follower_main();
}

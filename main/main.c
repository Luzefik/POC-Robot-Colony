#include "column_link.h"
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

/* ── Вибір ролі робота ────────────────────────────────────────────────────
 * Якщо перемикач SW1 не налаштований (CONFIG_UGV_MODE_SWITCH_GPIO = -1),
 * роль задається цим рядком. Поміняй значення і перезбери:
 *   0 = FOLLOWER (їде за ліхтариками), 1 = LEADER (керується геймпадом).
 * Якщо в menuconfig вказаний GPIO перемикача - рішення приймає він
 * (рівень 1 при старті = leader). */
#define DEFAULT_ROLE_LEADER 0

/* Черга на один результат детекції. Детекція пише через xQueueOverwrite
 * (нове завжди перетирає старе), контур керування читає. Так контур
 * ніколи не працює із застарілими координатами. */
QueueHandle_t dots_detection_queue;

/* Зупиняємось, коли середня відстань між ліхтариками (у пікселях)
 * менша за це число. */
#define STOP_GAP_PX 70

/* ── Стан геймпада (використовується driving-logic/my_platform.c) ──────── */

static volatile bool s_gamepad_connected;

void set_gamepad_connected(bool connected) {
    s_gamepad_connected = connected;
    ESP_LOGI(TAG, "Gamepad %s", connected ? "connected" : "disconnected");
    if (!connected)
        drive_stop(); // запобіжник: без пілота не їдемо
}

/* ── Задача детекції (ядро 1) ─────────────────────────────────────────────
 * Крутиться з частотою камери: бере найсвіжіший кадр, шукає три ліхтарики,
 * результат кладе в чергу. Раз на секунду пише fps у лог. */

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

        // Компактна статистика раз на секунду замість спаму на кожен кадр
        frames++;
        int64_t now = esp_timer_get_time();
        if (now - window_start >= 1000000) {
            ESP_LOGI(TAG, "Detection: %.1f fps",
                     frames * 1000000.0f / (now - window_start));
            frames = 0;
            window_start = now;
        }

        vTaskDelay(1);
    }
}

/* ── Режим FOLLOWER: керування моторами ──────────────────────────────────
 *
 * Оригінальна, відтюнена в полі логіка:
 *   acc  - середня відстань між сусідніми ліхтариками (пікселі).
 *          Більша відстань = лідер ближче.
 *   turn - зсув центрального ліхтарика від центру кадру по X.
 * Мертва зона моторів 351 PWM (менше редуктори не крутять),
 * розгін/гальмування кроком 4 за цикл, поворот кроком 4. */

typedef struct {
    int16_t turn;
    int16_t acc;
    bool valid;
} Conv;

/* Чекає (блокується), поки детекція не опублікує результат кадру -
 * тобто контур керування крокує з частотою камери. */
static Conv getData(void) {
    BlobResult dots;
    Conv result = {0, 0, false};

    if (xQueueReceive(dots_detection_queue, &dots, portMAX_DELAY) != pdTRUE)
        return result;
    if (!dots.valid)
        return result;

    result.valid = true;
    result.turn = (int16_t)dots.dots[1].x;

    // Відстані між сусідніми ліхтариками, середнє - оцінка дистанції
    float gap_1 = hypotf(dots.dots[0].x - dots.dots[1].x,
                         dots.dots[0].y - dots.dots[1].y);
    float gap_2 = hypotf(dots.dots[1].x - dots.dots[2].x,
                         dots.dots[1].y - dots.dots[2].y);
    result.acc = (int16_t)((gap_1 + gap_2) / 2.0f);

    ESP_LOGD(TAG, "gap_1=%.1f gap_2=%.1f acc=%d turn=%d", gap_1, gap_2,
             result.acc, result.turn);

    return result;
}

static void follower_loop(void) {
    static int16_t turn = 0;
    static int16_t speed = 0;

    for (;;) {
        Conv data = getData();

        // Ціль близько - стоїмо на місці
        if (data.valid && data.acc < STOP_GAP_PX) {
            ESP_LOGI(TAG, "TARGET NEAR (acc=%d) - STOPPING", data.acc);
            speed = 0;
            turn = 0;
            motor(0, 0);
            motor(1, 0);
            motor(2, 0);
            motor(3, 0);
            continue;
        }

        int16_t pacc = clampi(data.acc, -508, 508) / 8;
        int16_t px = clampi(data.turn, -508, 508) / 8;

        if (!data.valid) {
            // Пошук: крутимось у бік, де ціль бачили востаннє
            if (turn <= 0) {
                motor(0, 0);
                motor(1, 0);
                motor(2, 370);
                motor(3, 0);
            } else {
                motor(0, 370);
                motor(1, 0);
                motor(2, 0);
                motor(3, 0);
            }
            if (turn <= 0) {
                motor(0, 300 + px / 3);
                motor(1, 0);
                motor(2, 0);
                motor(3, 0);
            } else {
                motor(0, 0);
                motor(1, 0);
                motor(2, 300 + px / 3);
                motor(3, 0);
            }
        } else {
            // Компенсація мертвої зони редукторів
            if (pacc > 0) {
                pacc += 351;
            }

            // Плавний розгін/гальмування: не більше 4 PWM за цикл
            if (speed < pacc) {
                speed += 4;
            }
            if (speed > pacc) {
                speed -= 4;
            }

            // Нижче 351 мотори все одно не крутять - одразу піднімаємо
            if (0 < speed && speed < 351 && pacc) {
                speed = 351;
            }

            // Поворот теж плавно, кроком 4
            px = clampi(px, -64, 63);
            if (px > turn) {
                turn += 4;
            }
            if (px < turn) {
                turn -= 4;
            }

            // Мікс: ліві колеса = speed + turn, праві = speed - turn
            motor(0, speed + turn);
            motor(1, 0);
            motor(2, speed - turn);
            motor(3, 0);
        }

        ESP_LOGD(TAG, "MOTOR: turn=%d speed=%d valid=%d", turn, speed,
                 data.valid);
    }
}

static void follower_main(void) {
    ESP_LOGI(TAG, "Starting in FOLLOWER mode (vision tracking)");

#if CONFIG_UGV_ENABLE_WEB_STREAM
    /* Wi-Fi піднімаємо ДО камери: на свіжій платі phy_init пише
     * калібровку у flash, а запис у flash під час роботи DMA камери
     * викликає паніку "cache disabled but cached memory region accessed". */
    wifi_register_got_ip_cb(web_stream_start);
    wifi_init_sta();
#endif

    /* Зв'язок колони: фолловер поки що тільки слухає. Отримані
     * повідомлення видно в лозі та на сторінці /log. Якщо робот не в
     * хотспоті - сам знаходить канал відправника скануванням. */
    column_link_init(true);

    /* Відтюнені на /tune і збережені пороги детекції (якщо є в NVS) */
    dot_detection_params_load();

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

    // Зір - на ядрі 1, контур керування лишається на ядрі 0 (main task)
    xTaskCreatePinnedToCore(detection_task, "detection", 8192, NULL, 5, NULL, 1);

    follower_loop();
}

/* ── Режим LEADER: Bluetooth-геймпад ───────────────────────────────────── */

/* Демонстрація зв'язку: лідер розсилає лічений "ping" 5 разів на секунду,
 * фолловери в радіусі бачать його в лозі. 5 Гц - щоб автопошук каналу у
 * фолловерів був швидким. Далі за планом тут буде справжній пакет
 * телеметрії (documentation/ESPNOW_COLUMN_LINK_PLAN.md). */
static void column_tx_task(void *arg) {
    for (;;) {
        column_link_send_text("ping");
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void leader_main(void) {
#if CONFIG_BT_ENABLED
    ESP_LOGI(TAG, "Starting in LEADER mode (Bluetooth gamepad)");

    motor_init();

#if CONFIG_UGV_ENABLE_WEB_STREAM
    /* Стрім з лідера теж можливий. BT і Wi-Fi ділять одне радіо
     * (coexistence), тому під час перегляду стріму геймпад може трохи
     * лагати. Wi-Fi піднімаємо навіть якщо камера не завелась - щоб
     * зв'язок колони сидів на тому ж каналі хотспота, що й інші роботи. */
    if (camera_init_board() == ESP_OK)
        wifi_register_got_ip_cb(web_stream_start);
    else
        ESP_LOGW(TAG, "Camera init failed - no stream, Wi-Fi still up");
    wifi_init_sta();
#endif

    /* Відправник - "якір" каналу (канал хотспота або UGV_ESPNOW_CHANNEL).
     * Фолловери самі його знаходять, тому сканування тут не потрібне. */
    if (column_link_init(false) == ESP_OK)
        xTaskCreate(column_tx_task, "column_tx", 3072, NULL, 4, NULL);

    btstack_init();
    uni_platform_set_custom(get_my_platform());
    uni_init(0, NULL);

    btstack_run_loop_execute(); // звідси не повертається
#else
    ESP_LOGE(TAG, "Leader mode requires CONFIG_BT_ENABLED "
                  "(see sdkconfig.defaults); staying idle");
#endif
}

/* ── Точка входу ─────────────────────────────────────────────────────────
 * SW1 обирає роль при кожному включенні без перепрошивки (звіт, IV.B).
 * GPIO перемикача задається в menuconfig -> "UGV column configuration". */

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

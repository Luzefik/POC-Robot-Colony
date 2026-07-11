#include "column_link.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <string.h>

static const char *TAG = "column_link";

static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* Channel scan: dwell must cover at least one beacon period of the sender
 * (the PoC ping is sent every 200 ms). Rescan if the link goes silent. */
#define SCAN_DWELL_MS 500
#define RESCAN_AFTER_MS 10000

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static column_msg_t s_last_rx;
static int64_t s_last_rx_us = -1;
static uint16_t s_tx_counter;
static uint32_t s_rx_count;

/* ESP-NOW needs a started Wi-Fi driver. The debug stream may have started
 * it already (wifi_init_sta); otherwise bring up the bare minimum: STA mode,
 * not connected to anything, parked on channel 1. */
static esp_err_t ensure_wifi_started(void) {
    wifi_mode_t mode;
    if (esp_wifi_get_mode(&mode) == ESP_OK)
        return ESP_OK; // driver already initialized elsewhere

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
        return ret;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(
        esp_wifi_set_channel(CONFIG_UGV_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG, "Wi-Fi started for ESP-NOW only (channel %d, not connected)",
             CONFIG_UGV_ESPNOW_CHANNEL);
    return ESP_OK;
}

static bool sta_is_associated(void) {
    wifi_ap_record_t ap;
    return esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
}

/* Followers not joined to a hotspot don't know which channel the sender
 * sits on. Hop until traffic is heard; if the link then goes silent for
 * RESCAN_AFTER_MS, start hopping again. */
static void channel_scan_task(void *arg) {
    uint8_t ch = CONFIG_UGV_ESPNOW_CHANNEL;
    bool locked = false;

    for (;;) {
        if (sta_is_associated()) {
            /* AP dictates the channel now; nothing to scan. */
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        int64_t last_rx;
        taskENTER_CRITICAL(&s_mux);
        last_rx = s_last_rx_us;
        taskEXIT_CRITICAL(&s_mux);

        int64_t silence_ms =
            (last_rx < 0) ? INT64_MAX : (esp_timer_get_time() - last_rx) / 1000;

        if (silence_ms < RESCAN_AFTER_MS) {
            if (!locked) {
                uint8_t cur;
                wifi_second_chan_t sc;
                esp_wifi_get_channel(&cur, &sc);
                ESP_LOGI(TAG, "Column traffic found on channel %d - locked", cur);
                locked = true;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (locked) {
            ESP_LOGW(TAG, "Column link silent for %d ms - scanning channels",
                     RESCAN_AFTER_MS);
            locked = false;
        }

        ch = (ch % 13) + 1;
        esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
        vTaskDelay(pdMS_TO_TICKS(SCAN_DWELL_MS));
    }
}

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data,
                    int len) {
    if (len < (int)sizeof(column_msg_t))
        return;

    const column_msg_t *msg = (const column_msg_t *)data;
    if (msg->magic != COLUMN_MSG_MAGIC || msg->version != COLUMN_MSG_VERSION)
        return;

    taskENTER_CRITICAL(&s_mux);
    s_last_rx = *msg;
    s_last_rx.text[sizeof(s_last_rx.text) - 1] = '\0';
    s_last_rx_us = esp_timer_get_time();
    uint32_t rx_count = ++s_rx_count;
    taskEXIT_CRITICAL(&s_mux);

    /* First message and then every 10th - the PoC sender pings at 5 Hz,
     * logging each one would flood the console. */
    if (rx_count == 1 || rx_count % 10 == 0)
        ESP_LOGI(TAG,
                 "COLUMN RX from %02x:%02x:%02x:%02x:%02x:%02x: #%u \"%s\" "
                 "(%lu received)",
                 info->src_addr[0], info->src_addr[1], info->src_addr[2],
                 info->src_addr[3], info->src_addr[4], info->src_addr[5],
                 msg->counter, s_last_rx.text, (unsigned long)rx_count);
}

static void on_sent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS)
        ESP_LOGW(TAG, "Broadcast TX failed");
}

esp_err_t column_link_init(bool auto_channel_scan) {
    esp_err_t err = ensure_wifi_started();
    if (err != ESP_OK)
        return err;

    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init: %s", esp_err_to_name(err));
        return err;
    }

    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_sent));

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, BROADCAST_MAC, sizeof(BROADCAST_MAC));
    peer.channel = 0; // 0 = current Wi-Fi channel
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    if (auto_channel_scan)
        xTaskCreate(channel_scan_task, "column_scan", 3072, NULL, 3, NULL);

    uint8_t ch;
    wifi_second_chan_t sc;
    esp_wifi_get_channel(&ch, &sc);
    ESP_LOGI(TAG, "Column link up (ESP-NOW broadcast, channel %d%s)", ch,
             auto_channel_scan ? ", auto-scan on" : "");
    return ESP_OK;
}

esp_err_t column_link_send_text(const char *text) {
    column_msg_t msg = {
        .magic = COLUMN_MSG_MAGIC,
        .version = COLUMN_MSG_VERSION,
        .counter = ++s_tx_counter,
        .uptime_ms = (uint32_t)(esp_timer_get_time() / 1000),
    };
    snprintf(msg.text, sizeof(msg.text), "%s", text ? text : "");

    return esp_now_send(BROADCAST_MAC, (const uint8_t *)&msg, sizeof(msg));
}

bool column_link_get_last(column_msg_t *out, uint32_t *age_ms) {
    taskENTER_CRITICAL(&s_mux);
    bool have = s_last_rx_us >= 0;
    if (have && out)
        *out = s_last_rx;
    int64_t rx_us = s_last_rx_us;
    taskEXIT_CRITICAL(&s_mux);

    if (have && age_ms)
        *age_ms = (uint32_t)((esp_timer_get_time() - rx_us) / 1000);
    return have;
}

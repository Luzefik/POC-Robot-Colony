#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/* Minimal robot-to-robot link (ESP-NOW broadcast, proof of concept).
 *
 * One message type for now: a counted "ping" with a short text. All robots
 * on the SAME Wi-Fi channel hear every broadcast: either join the same
 * hotspot (debug stream enabled), or run all robots without Wi-Fi STA -
 * then everyone sits on the default channel 1.
 *
 * Full protocol design: documentation/ESPNOW_COLUMN_LINK_PLAN.md */

#define COLUMN_MSG_MAGIC 0xC0
#define COLUMN_MSG_VERSION 1

typedef struct __attribute__((packed)) {
    uint8_t magic;     // COLUMN_MSG_MAGIC
    uint8_t version;   // COLUMN_MSG_VERSION
    uint16_t counter;  // increments per sent message
    uint32_t uptime_ms; // sender uptime
    char text[16];     // zero-terminated demo payload
} column_msg_t;

/* Bring up ESP-NOW (starts minimal Wi-Fi if nothing started it before).
 *
 * auto_channel_scan = true (followers): if the robot is not associated to
 * an access point, hop through channels 1-13 until column traffic is heard,
 * then stay there - no manual channel coordination needed. Senders and
 * robots joined to a hotspot keep their channel fixed. */
esp_err_t column_link_init(bool auto_channel_scan);

/* Broadcast one message to every robot in range. */
esp_err_t column_link_send_text(const char *text);

/* Last received message; false if nothing arrived yet.
 * age_ms (optional, may be NULL) = how long ago it arrived. */
bool column_link_get_last(column_msg_t *out, uint32_t *age_ms);

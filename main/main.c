// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java:
// https://pvs-studio.com

#include "camera_pinout.h"
#include "dot_detection.h"
#include "driving.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "take_picture.h"
#include "web_stream.h"
#include "wifi_connect.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "app";

static void on_wifi_ready(void) {
  ESP_LOGI(TAG, "Wi-Fi ready → starting WEB STREAM");
  web_stream_start();
}

QueueHandle_t dots_detection_queue;
typedef struct {
  int16_t turn;
  int16_t acc;
  bool valid;
} Conv;

Conv getData() {
  BlobResult dots = {0};
  Conv result = {0, 0, false};

  if (xQueueReceive(dots_detection_queue, &dots, portMAX_DELAY)) {
    if (dots.blobs[0].count > 0 && dots.blobs[1].count > 0 &&
        dots.blobs[2].count > 0) {
      ESP_LOGI(TAG, "Valid data from queue");
      result.valid = true;
    } else {
      ESP_LOGW(TAG, "Invalid data - blobs have zero count");
      result.valid = false;
      return result;
    }
  } else {
    ESP_LOGW(TAG, "Queue empty");
    result.valid = false;
    return result;
  }

  result.turn = (int16_t)dots.blobs[1].cord_x;

  float gap_1_y = sqrt(pow(dots.blobs[0].cord_x - dots.blobs[1].cord_x, 2) +
                       pow(dots.blobs[0].cord_y - dots.blobs[1].cord_y, 2));
  float gap_2_y = sqrt(pow(dots.blobs[1].cord_x - dots.blobs[2].cord_x, 2) +
                       pow(dots.blobs[1].cord_y - dots.blobs[2].cord_y, 2));

  ESP_LOGI(TAG, "LEFT DOT:   X=%.1f, Y=%.1f", dots.blobs[0].cord_x,
           dots.blobs[0].cord_y);
  ESP_LOGI(TAG, "CENTER DOT: X=%.1f, Y=%.1f", dots.blobs[1].cord_x,
           dots.blobs[1].cord_y);
  ESP_LOGI(TAG, "RIGHT DOT:  X=%.1f, Y=%.1f", dots.blobs[2].cord_x,
           dots.blobs[2].cord_y);

  float gap_y = (gap_1_y + gap_2_y) / 2.0f;

  ESP_LOGI(TAG, "gap_1_y: %.1f, gap_2_y: %.1f, gap_y %.1f", gap_1_y, gap_2_y,
           gap_y);

  // if (gap_y < 75) {
  //     gap_y = 0;
  // }

  result.acc = gap_y;

  return result;
}

int16_t clamp(int x, int min, int max) {
  if (x < min) {
    return min;
  } else if (x >= max) {
    return max;
  }

  return x;
}

static void detection_task(void *arg) {
  while (1) {
    int64_t start_camera = esp_timer_get_time();

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }

    int64_t start_algo = esp_timer_get_time();
    process_image(fb);
    int64_t end_algo = esp_timer_get_time();

    esp_camera_fb_return(fb);

    int64_t end_camera = esp_timer_get_time();

    ESP_LOGI(
        TAG,
        "CAMERA TIMING - Total: %lld ms, Algo: %lld ms, Get+Return: %lld ms",
        (end_camera - start_camera) / 1000, (end_algo - start_algo) / 1000,
        (end_camera - start_camera - (end_algo - start_algo)) / 1000);

    vTaskDelay(1);
  }
}

void app_main(void) {
  // esp_err_t ret = nvs_flash_init();

  // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret ==
  // ESP_ERR_NVS_NEW_VERSION_FOUND) {
  //     ESP_ERROR_CHECK(nvs_flash_erase());
  //     ESP_ERROR_CHECK(nvs_flash_init());
  // }
  // ESP_ERROR_CHECK(ret);

  if (camera_init_board() != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed");
    return;
  }

  ESP_LOGI(TAG, "Camera initialized");

  dots_detection_queue = xQueueCreate(1, sizeof(BlobResult));
  if (dots_detection_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create dots_detection_queue");
    return;
  }
  ESP_LOGI(TAG, "Queue created successfully");

  motor_init();
  ESP_LOGI(TAG, "Motors initialized");

  xTaskCreate(detection_task, "detection", 8192, NULL, 5, NULL);

  // lcd_status_print(0);

  // wifi_register_got_ip_cb(on_wifi_ready);
  // wifi_init_sta();

  static int16_t turn = 0;
  static int16_t speed = 0;
  static bool prev_detected = false;
  int64_t start_loop = esp_timer_get_time();
  // float Kp_turn = 1.5;
  // float Kp_speed = 1.0;
  for (;;) {
    // -512 <-> 512
    // 508 бо джойстик у нульовій позиціє для X та Y маюьть по 4 одиниці
    Conv data = getData();
    int64_t after_getData = esp_timer_get_time();

    if (data.valid && data.acc < 70) {
      ESP_LOGI(TAG, "TARGET NEAR (y=%d) - STOPPING", data.acc);
      speed = 0;
      turn = 0;
      motor(0, 0);
      motor(1, 0);
      motor(2, 0);
      motor(3, 0);
      start_loop = esp_timer_get_time();
      continue;
    }

    int16_t pacc = clamp(data.acc, -508, 508) / 8;
    int16_t px = clamp(data.turn, -508, 508) / 8;
    // static int16_t lastc;
    // if (data.valid) {lastc = px/25;}

    int64_t curr_timer = esp_timer_get_time();
    if (curr_timer - start_loop > 50) {
      ESP_LOGW(TAG, "LOOP TIME EXCEEDED: %lld ms",
               (curr_timer - start_loop) / 1000);
      ESP_LOGW(TAG, "Data: %lld", curr_timer - start_loop);
    }
    static bool flag = 0;
    if (!data.valid) {
      // if (flag < 3) {
      //     motor(0, 0);
      //     motor(1, 0);
      //     motor(2, 0);
      //     motor(3, 0);
      //     flag++;
      // } else {
      //     // flag += abs(lastc);
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
      flag = 1;
      if (flag) {
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
        flag = 0;
      }
    } else {
      if (pacc > 0) {
        pacc += 351;
      }

      if (speed < pacc) {
        speed += 4;
      }
      if (speed > pacc) {
        speed -= 4;
      }

      if (0 < speed && speed < 351 && pacc) {
        speed = 351;
      }
      px = clamp(px, -64, 63);
      if (px > turn) {
        turn += 4;
      }
      if (px < turn) {
        turn -= 4;
      }

      motor(0, speed + (turn));
      motor(1, 0);
      motor(2, speed - (turn));
      motor(3, 0);
    }

    int64_t end_loop = esp_timer_get_time();
    int64_t loop_duration = (end_loop - start_loop) / 1000;
    int64_t getData_time = (after_getData - start_loop) / 1000;

    ESP_LOGI(TAG,
             "MOTOR LOOP - Total: %lld ms, getData: %lld ms, Turn: %d, Speed: "
             "%d, Valid: %d",
             loop_duration, getData_time, turn, speed, data.valid);

    start_loop = esp_timer_get_time();
    // vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}
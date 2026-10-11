/*
 * step5_merged/main.cpp — объединение Step 3 (wake word "Ирина" на реальном
 * микрофоне) и Step 4 (OLED: диагностика при старте + глазки с эмоциями).
 *
 * Архитектура: два параллельных FreeRTOS-таска на разных ядрах ESP32-S3,
 * связанные одним volatile-флагом (это нормально для одного bool,
 * устанавливаемого в одном месте и читаемого в другом — без гонки данных
 * на уровне отдельных операций чтения/записи int; для более сложного обмена
 * данными между задачами в будущем используйте очередь FreeRTOS, не флаги):
 *
 *   - wakeWordTask (ядро 1): читает I2S-микрофон, кормит детектор. При
 *     срабатывании выставляет s_wake_event = true. Никогда не трогает экран.
 *
 *   - eyesTask (ядро 0): ~30 раз/сек обновляет анимацию глаз. Видит
 *     s_wake_event -> кратко показывает эмоцию SURPRISED + моргает, потом
 *     возвращается в NEUTRAL. Единственная задача, которая трогает OLED —
 *     так не нужен мьютекс на framebuffer.
 *
 * Диагностика при старте (SPEAKER/WIFI/SERVER) выполняется один раз в
 * app_main() ДО запуска обеих задач — WIFI/SERVER пока заглушки (реальные
 * появятся в step6 вместе с WiFi/OTA).
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2s_std.h"
#include "driver/uart.h"

#include "wake_word.h"
#include "wake_word_model.h"   // сгенерировать из irina.tflite, см. ниже
#include "ssd1306.h"
#include "eyes.h"

static const char* TAG = "step5";

// =====================================================================
//           !!! ЗАПОЛНИТЕ ПОД ВАШУ ПЛАТУ — ОБЕ ГРУППЫ ПИНОВ !!!
// =====================================================================
#ifndef WW_PINS_CONFIGURED
#error "Заполните WW_I2S_BCLK_PIN/WW_I2S_WS_PIN/WW_I2S_DIN_PIN (микрофон, " \
       "как в Step 3) и определите WW_PINS_CONFIGURED."
#endif
#ifndef OLED_PINS_CONFIGURED
#error "Заполните OLED_SDA_PIN/OLED_SCL_PIN (экран, как в Step 4) и " \
       "определите OLED_PINS_CONFIGURED."
#endif

// Примеры (РАСКОММЕНТИРУЙТЕ И ИСПРАВЬТЕ под свою плату, удалите #error выше):
// #define WW_PINS_CONFIGURED
// #define WW_I2S_BCLK_PIN   GPIO_NUM_4
// #define WW_I2S_WS_PIN     GPIO_NUM_5
// #define WW_I2S_DIN_PIN    GPIO_NUM_6
//
// #define OLED_PINS_CONFIGURED
// #define OLED_SDA_PIN      GPIO_NUM_8
// #define OLED_SCL_PIN      GPIO_NUM_9
#define OLED_I2C_ADDR  0x3C

// Параметры модели "Ирина" — из её irina.json (как в Step 3).
#define WW_TENSOR_ARENA_SIZE    (26080 + 2048)
#define WW_PROBABILITY_CUTOFF   0.97f
#define WW_SLIDING_WINDOW       5
#define I2S_READ_SAMPLES        320   // 320 сэмплов @16кГц = 20мс

static i2s_chan_handle_t s_rx_handle = nullptr;
static WakeWordDetector  s_detector;
static volatile bool     s_wake_event = false;

// --- I2S микрофон (как в Step 3) ---

static bool initMic() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  if (i2s_new_channel(&chan_cfg, nullptr, &s_rx_handle) != ESP_OK) {
    ESP_LOGE(TAG, "i2s_new_channel failed");
    return false;
  }

  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(WW_SAMPLE_RATE),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
          I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
          .mclk = I2S_GPIO_UNUSED,
          .bclk = WW_I2S_BCLK_PIN,
          .ws   = WW_I2S_WS_PIN,
          .dout = I2S_GPIO_UNUSED,
          .din  = WW_I2S_DIN_PIN,
          .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false},
      },
  };
  // Если L/R-пин INMP441 подключён к VDD, а не к GND — поменяйте на RIGHT.
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  if (i2s_channel_init_std_mode(s_rx_handle, &std_cfg) != ESP_OK) {
    ESP_LOGE(TAG, "i2s_channel_init_std_mode failed");
    return false;
  }
  if (i2s_channel_enable(s_rx_handle) != ESP_OK) {
    ESP_LOGE(TAG, "i2s_channel_enable failed");
    return false;
  }
  ESP_LOGI(TAG, "I2S микрофон запущен");
  return true;
}

// См. комментарий в Step 3: сдвиг на 14 — приблизительное практическое
// значение для INMP441, проверяйте по RMS и подстраивайте при необходимости.
static inline int16_t i2s32ToPcm16(int32_t raw) {
  int32_t v = raw >> 14;
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}

// --- Задача распознавания wake word (ядро 1) ---

static void wakeWordTask(void* arg) {
  int32_t i2s_raw[I2S_READ_SAMPLES];
  int16_t pcm[I2S_READ_SAMPLES];

  for (;;) {
    size_t bytes_read = 0;
    esp_err_t err = i2s_channel_read(s_rx_handle, i2s_raw, sizeof(i2s_raw),
                                      &bytes_read, pdMS_TO_TICKS(100));
    if (err != ESP_OK || bytes_read == 0) continue;

    size_t got = bytes_read / sizeof(int32_t);
    for (size_t i = 0; i < got; i++) pcm[i] = i2s32ToPcm16(i2s_raw[i]);

    if (s_detector.process(pcm, got)) {
      ESP_LOGW(TAG, ">>> WAKE WORD DETECTED <<<");
      s_wake_event = true;
    }
  }
}

// --- Задача анимации глаз (ядро 0) ---

static void eyesTask(void* arg) {
  int64_t surprisedUntilUs = 0;

  for (;;) {
    if (s_wake_event) {
      s_wake_event = false;
      eyes_set_emotion(Emotion::SURPRISED);
      eyes_blink();
      surprisedUntilUs = esp_timer_get_time() + 1500000;  // 1.5с
      ESP_LOGI(TAG, "Глазки: реакция на детект wake word");
    }
    if (surprisedUntilUs != 0 && esp_timer_get_time() > surprisedUntilUs) {
      eyes_set_emotion(Emotion::NEUTRAL);
      surprisedUntilUs = 0;
    }

    eyes_update();
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

// --- "диагностика" при старте (SPEAKER/WIFI/SERVER — пока заглушки) ---

static bool checkSpeaker() { vTaskDelay(pdMS_TO_TICKS(400)); return true; }  // TODO: step6
static bool checkWifi()    { vTaskDelay(pdMS_TO_TICKS(600)); return true; }  // TODO: step6
static bool checkServer()  { vTaskDelay(pdMS_TO_TICKS(500)); return true; }  // TODO: step6

struct DiagStep { const char* label; bool (*check)(); };
static DiagStep DIAG_STEPS[] = {
  {"SPEAKER", checkSpeaker},
  {"WIFI",    checkWifi},
  {"SERVER",  checkServer},
};

static void runBootDiagnostics() {
  const int lineH = 10;
  int y = 4;

  ssd1306_clear();
  ssd1306_draw_string(4, y, "STARTING...", true, 1);
  ssd1306_flush();
  y += lineH + 2;
  vTaskDelay(pdMS_TO_TICKS(300));

  for (auto& step : DIAG_STEPS) {
    char line[32];
    snprintf(line, sizeof(line), "%s: ...", step.label);
    ssd1306_draw_string(4, y, line, true, 1);
    ssd1306_flush();
    ESP_LOGI(TAG, "%s", line);

    bool ok = step.check();

    ssd1306_fill_rect(4, y, OLED_WIDTH - 8, 8, false);
    snprintf(line, sizeof(line), "%s: %s", step.label, ok ? "OK" : "FAIL");
    ssd1306_draw_string(4, y, line, true, 1);
    ssd1306_flush();
    ESP_LOGI(TAG, "%s", line);

    if (!ok) {
      ssd1306_draw_string(4, y + lineH, "CHECK FAILED", true, 1);
      ssd1306_flush();
      ESP_LOGE(TAG, "Диагностика остановлена на '%s'", step.label);
      for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    y += lineH;
    vTaskDelay(pdMS_TO_TICKS(150));
  }
  vTaskDelay(pdMS_TO_TICKS(400));
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "=== Step 5: wake word + глазки (без WiFi, см. step6) ===");

  uart_driver_install(UART_NUM_0, 256, 0, 0, nullptr, 0);

  if (!ssd1306_init(OLED_SDA_PIN, OLED_SCL_PIN, OLED_I2C_ADDR)) {
    ESP_LOGE(TAG, "OLED не инициализировался — дальше нет смысла продолжать.");
    return;
  }

  runBootDiagnostics();

  EyeConfig cfg;
  cfg.left_cx = 38;  cfg.left_cy = 32;
  cfg.right_cx = 90; cfg.right_cy = 32;
  cfg.radius = 20;
  cfg.pupil_radius = 5;
  eyes_init(cfg);

  if (!initMic()) {
    ESP_LOGE(TAG, "Микрофон не запустился — глазки продолжат работать, "
             "но реагировать на голос не будут.");
  } else {
    WakeWordConfig wwcfg;
    wwcfg.model_data          = g_wake_word_model_data;
    wwcfg.tensor_arena_size   = WW_TENSOR_ARENA_SIZE;
    wwcfg.probability_cutoff  = WW_PROBABILITY_CUTOFF;
    wwcfg.sliding_window_size = WW_SLIDING_WINDOW;
    wwcfg.name                = WAKE_WORD_NAME;

    if (!s_detector.begin(wwcfg)) {
      ESP_LOGE(TAG, "Детектор wake word не инициализировался.");
    } else {
      // Ядро 1: распознавание (нагруженная TFLM-инференсом задача).
      xTaskCreatePinnedToCore(wakeWordTask, "wake_word", 8192, nullptr, 5, nullptr, 1);
    }
  }

  // Ядро 0: анимация глаз — лёгкая задача, но должна быть отзывчивой
  // (30 кадров/сек), поэтому отдельное ядро от TFLM-инференса на ядре 1.
  xTaskCreatePinnedToCore(eyesTask, "eyes", 4096, nullptr, 3, nullptr, 0);

  ESP_LOGI(TAG, "Готово. Скажите 'Ирина' — глазки должны среагировать.");
}

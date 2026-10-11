/*
 * wake_word.h — детектор wake word для ESP32-S3 на базе microWakeWord.
 *
 * ИСПРАВЛЕНО: изначально файл был написан под Arduino IDE (предполагалось,
 * что Arduino Core 3.x включает esp-tflite-micro). Это предположение
 * оказалось неверным (см. STEP1/STEP2_GUIDE.md) — проект уже переехал на
 * PlatformIO + чистый framework=espidf, и этот файл теперь тоже под него:
 * Arduino.h убран, Serial.* заменены на обычный printf (в ESP-IDF он уже
 * подключён к UART0 через newlib, ничего настраивать не нужно).
 *
 * КОНВЕЙЕР (полностью на устройстве, наружу ничего не уходит):
 *   I2S 16кГц int16
 *     -> микрофронтенд TFLM (окно 30мс, шаг 10мс, 40 мел-каналов 125..7500 Гц,
 *        с шумоподавлением и авто-усилением)
 *     -> 40 признаков uint16 (диапазон примерно 0..670)
 *     -> /25.6 -> квантование в int8 по scale/zero_point входного тензора
 *     -> TFLite-интерпретатор -> вероятность
 *     -> скользящее среднее по N окнам -> порог -> детект
 *
 * ВАЖНО ПРО МАСШТАБИРОВАНИЕ: делитель 25.6 — не произвольный. При обучении
 * microWakeWord выход фронтенда делится именно на 25.6, чтобы получить float
 * в диапазоне ~0..26. Если его не применить, модель получит мусор на входе и
 * будет молча не срабатывать — без единой ошибки в логе. Это самая частая
 * причина «модель не работает».
 *
 * ЧЕСТНОЕ ПРЕДУПРЕЖДЕНИЕ: этот код написан по документации API и НЕ был
 * скомпилирован и проверен на железе — у меня нет ни платы, ни тулчейна.
 * Ожидайте правок при первой сборке, особенно в списке регистрируемых
 * операций (registerOps) и в размере арены. Логика конвейера и числовые
 * константы выверены по исходникам ESPHome и спецификации моделей.
 */

#ifndef WAKE_WORD_H
#define WAKE_WORD_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Параметры конвейера признаков. Менять нельзя — они зашиты в то,
// как обучалась модель.
#define WW_SAMPLE_RATE        16000
#define WW_FEATURE_COUNT      40      // мел-каналов на срез
#define WW_WINDOW_MS          30      // длительность окна анализа
#define WW_STRIDE_MS          10      // шаг между окнами
#define WW_SAMPLES_PER_STRIDE (WW_SAMPLE_RATE * WW_STRIDE_MS / 1000)  // 160

// Исторический делитель выхода фронтенда (см. комментарий выше).
#define WW_FEATURE_DIVISOR    25.6f

// Сколько первых срезов игнорировать после старта/сброса: пока фронтенд
// не оценил уровень шума, вероятности мусорные и дают ложные срабатывания.
#define WW_IGNORE_SLICES      20

struct WakeWordConfig {
  const unsigned char* model_data;   // .tflite как C-массив
  size_t tensor_arena_size;          // обычно 24-60 КБ, зависит от модели
  float  probability_cutoff;         // порог, напр. 0.97
  uint8_t sliding_window_size;       // усреднение, обычно 5
  const char* name;                  // для логов
};

class WakeWordDetector {
 public:
  WakeWordDetector();
  ~WakeWordDetector();

  // Выделяет арену (в PSRAM), поднимает фронтенд и интерпретатор.
  bool begin(const WakeWordConfig& cfg);

  // Скармливает аудио. Возвращает true РОВНО ОДИН РАЗ в момент детекта.
  // samples/count — сырой PCM16 с микрофона (моно, 16 кГц).
  // Можно звать с любым размером буфера: лишнее копится внутри.
  bool process(const int16_t* samples, size_t count);

  // Сбрасывает скользящее окно и состояние фронтенда. Звать ПОСЛЕ детекта
  // и после воспроизведения, чтобы не ловить хвосты.
  void reset();

  float lastProbability() const { return last_probability_; }
  bool  isReady() const { return ready_; }

 private:
  bool generateFeatures(const int16_t* samples, size_t count,
                        size_t* consumed, int8_t out[WW_FEATURE_COUNT]);
  bool runInference(const int8_t features[WW_FEATURE_COUNT], float* probability);

  void*  impl_ = nullptr;      // прячем TFLM-типы, чтобы не тащить их в .h
  bool   ready_ = false;
  float  last_probability_ = 0.0f;
  int16_t ignore_slices_ = WW_IGNORE_SLICES;

  // Накопитель для «хвоста» сэмплов, не добравших до полного шага.
  int16_t pending_[WW_SAMPLES_PER_STRIDE * 4];
  size_t  pending_count_ = 0;

  float   prob_window_[16];
  uint8_t prob_window_size_ = 5;
  uint8_t prob_window_idx_ = 0;
  uint8_t prob_window_filled_ = 0;
  float   cutoff_ = 0.9f;
  const char* name_ = "wake";
};

#endif  // WAKE_WORD_H

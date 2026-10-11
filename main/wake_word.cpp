/*
 * wake_word.cpp — реализация детектора microWakeWord для ESP32-S3.
 * См. подробные комментарии в wake_word.h.
 */

#include "wake_word.h"
#include "esp_heap_caps.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

// ИСПРАВЛЕНО: это НЕ путь из основного репозитория TensorFlow (там этого
// компонента для ESP-IDF просто нет). Правильные заголовки даёт компонент
// esphome/esp-micro-speech-features — тот же, что уже подтверждён рабочим
// на вашем железе в Step 2 (STEP2_GUIDE.md). Добавьте зависимость в
// main/idf_component.yml:
//   esphome/esp-micro-speech-features: "^1.2.3"
#include "frontend.h"
#include "frontend_util.h"

#include <math.h>
#include <algorithm>  // std::min вместо Arduino-шного min()

namespace {

// Резолвер операций для streaming-моделей microWakeWord.
// Важно: VarHandle / ReadVariable / AssignVariable / CallOnce обязательны —
// именно через resource variables модель хранит своё streaming-состояние
// между вызовами Invoke(). Без них модель либо не аллоцируется, либо
// ведёт себя как non-streaming и не детектит ничего.
using WWOpResolver = tflite::MicroMutableOpResolver<21>;

bool registerOps(WWOpResolver& r) {
  if (r.AddCallOnce()          != kTfLiteOk) return false;
  if (r.AddVarHandle()         != kTfLiteOk) return false;
  if (r.AddReadVariable()      != kTfLiteOk) return false;
  if (r.AddAssignVariable()    != kTfLiteOk) return false;
  if (r.AddConv2D()            != kTfLiteOk) return false;
  if (r.AddDepthwiseConv2D()   != kTfLiteOk) return false;
  if (r.AddFullyConnected()    != kTfLiteOk) return false;
  if (r.AddReshape()           != kTfLiteOk) return false;
  if (r.AddConcatenation()     != kTfLiteOk) return false;
  if (r.AddStridedSlice()      != kTfLiteOk) return false;
  if (r.AddSplitV()            != kTfLiteOk) return false;  // подтверждено в реальном графе irina.tflite
  if (r.AddAdd()               != kTfLiteOk) return false;
  if (r.AddMul()               != kTfLiteOk) return false;
  if (r.AddMean()              != kTfLiteOk) return false;
  if (r.AddLogistic()          != kTfLiteOk) return false;
  if (r.AddQuantize()          != kTfLiteOk) return false;
  if (r.AddDequantize()        != kTfLiteOk) return false;
  if (r.AddAveragePool2D()     != kTfLiteOk) return false;
  if (r.AddMaxPool2D()         != kTfLiteOk) return false;
  if (r.AddPad()               != kTfLiteOk) return false;
  if (r.AddRelu()              != kTfLiteOk) return false;
  return true;
}

struct WWImpl {
  FrontendConfig  frontend_config;
  FrontendState   frontend_state;
  uint8_t*        arena = nullptr;
  size_t          arena_size = 0;
  WWOpResolver    resolver;
  tflite::MicroInterpreter* interpreter = nullptr;
  TfLiteTensor*   input = nullptr;
  TfLiteTensor*   output = nullptr;
  float           input_scale = 1.0f;
  int             input_zero_point = 0;
  bool            frontend_ready = false;
};

}  // namespace

WakeWordDetector::WakeWordDetector() {
  memset(prob_window_, 0, sizeof(prob_window_));
  memset(pending_, 0, sizeof(pending_));
}

WakeWordDetector::~WakeWordDetector() {
  WWImpl* impl = static_cast<WWImpl*>(impl_);
  if (impl) {
    if (impl->interpreter) { delete impl->interpreter; impl->interpreter = nullptr; }
    if (impl->frontend_ready) FrontendFreeStateContents(&impl->frontend_state);
    if (impl->arena) { heap_caps_free(impl->arena); impl->arena = nullptr; }
    delete impl;
    impl_ = nullptr;
  }
}

bool WakeWordDetector::begin(const WakeWordConfig& cfg) {
  name_ = cfg.name ? cfg.name : "wake";
  cutoff_ = cfg.probability_cutoff;
  prob_window_size_ = cfg.sliding_window_size;
  if (prob_window_size_ == 0) prob_window_size_ = 1;
  if (prob_window_size_ > 16) prob_window_size_ = 16;

  WWImpl* impl = new WWImpl();
  impl_ = impl;

  // --- 1. Микрофронтенд ---
  // Эти значения обязаны совпадать с теми, на которых обучалась модель.
  FrontendConfig& fc = impl->frontend_config;
  fc.window.size_ms                        = WW_WINDOW_MS;
  fc.window.step_size_ms                   = WW_STRIDE_MS;
  fc.filterbank.num_channels               = WW_FEATURE_COUNT;
  fc.filterbank.lower_band_limit           = 125.0f;
  fc.filterbank.upper_band_limit           = 7500.0f;
  fc.noise_reduction.smoothing_bits        = 10;
  fc.noise_reduction.even_smoothing        = 0.025f;
  fc.noise_reduction.odd_smoothing         = 0.06f;
  fc.noise_reduction.min_signal_remaining  = 0.05f;
  fc.pcan_gain_control.enable_pcan         = 1;
  fc.pcan_gain_control.strength            = 0.95f;
  fc.pcan_gain_control.offset              = 80.0f;
  fc.pcan_gain_control.gain_bits           = 21;
  fc.log_scale.enable_log                  = 1;
  fc.log_scale.scale_shift                 = 6;

  if (!FrontendPopulateState(&fc, &impl->frontend_state, WW_SAMPLE_RATE)) {
    printf("[WW] Не удалось инициализировать микрофронтенд" "\n");
    return false;
  }
  impl->frontend_ready = true;

  // --- 2. Арена в PSRAM ---
  // Держим её в PSRAM: внутренней RAM на S3 мало, а там уже живут
  // I2S-буферы, WiFi-стек и очереди.
  impl->arena_size = cfg.tensor_arena_size;
  impl->arena = static_cast<uint8_t*>(
      heap_caps_malloc(impl->arena_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!impl->arena) {
    printf("[WW] PSRAM недоступна, пробую внутреннюю RAM" "\n");
    impl->arena = static_cast<uint8_t*>(malloc(impl->arena_size));
  }
  if (!impl->arena) {
    printf("[WW] Не хватило памяти на арену (%u байт)\n",
                  (unsigned)impl->arena_size);
    return false;
  }

  // --- 3. Модель и интерпретатор ---
  const tflite::Model* model = tflite::GetModel(cfg.model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    printf("[WW] Несовпадение версии схемы: модель %lu, библиотека %d\n",
                  (unsigned long)model->version(), TFLITE_SCHEMA_VERSION);
    return false;
  }

  if (!registerOps(impl->resolver)) {
    printf("[WW] Не удалось зарегистрировать операции" "\n");
    return false;
  }

  impl->interpreter = new tflite::MicroInterpreter(
      model, impl->resolver, impl->arena, impl->arena_size);

  if (impl->interpreter->AllocateTensors() != kTfLiteOk) {
    // Самая частая причина — маленькая арена. Сообщаем понятно.
    printf("[WW] AllocateTensors не удался. Увеличьте tensor_arena_size "
                  "(сейчас %u байт).\n", (unsigned)impl->arena_size);
    return false;
  }

  impl->input  = impl->interpreter->input(0);
  impl->output = impl->interpreter->output(0);

  // Параметры квантования берём ИЗ МОДЕЛИ, а не хардкодим: у разных
  // моделей они отличаются, и хардкод — тихий источник неработающего детекта.
  impl->input_scale      = impl->input->params.scale;
  impl->input_zero_point = impl->input->params.zero_point;

  if (impl->input_scale <= 0.0f) {
    printf("[WW] У входного тензора некорректный scale — модель несовместима" "\n");
    return false;
  }

  printf("[WW] '%s' готов. Арена: %u байт, использовано: %u. "
                "Вход scale=%.5f zp=%d, порог=%.2f, окно=%u\n",
                name_, (unsigned)impl->arena_size,
                (unsigned)impl->interpreter->arena_used_bytes(),
                impl->input_scale, impl->input_zero_point,
                cutoff_, prob_window_size_);

  ready_ = true;
  reset();
  return true;
}

void WakeWordDetector::reset() {
  WWImpl* impl = static_cast<WWImpl*>(impl_);
  if (impl && impl->frontend_ready) FrontendReset(&impl->frontend_state);
  memset(prob_window_, 0, sizeof(prob_window_));
  prob_window_idx_ = 0;
  prob_window_filled_ = 0;
  pending_count_ = 0;
  ignore_slices_ = WW_IGNORE_SLICES;
  last_probability_ = 0.0f;
}

bool WakeWordDetector::generateFeatures(const int16_t* samples, size_t count,
                                        size_t* consumed,
                                        int8_t out[WW_FEATURE_COUNT]) {
  WWImpl* impl = static_cast<WWImpl*>(impl_);
  size_t processed = 0;
  FrontendOutput fo = FrontendProcessSamples(
      &impl->frontend_state, const_cast<int16_t*>(samples), count, &processed);
  *consumed = processed;

  if (fo.size == 0) return false;  // ещё не набралось на полное окно

  for (size_t i = 0; i < WW_FEATURE_COUNT && i < fo.size; i++) {
    // Выход фронтенда ~0..670 -> делим на 25.6 -> ~0..26 (так обучали)
    // -> квантуем параметрами входного тензора.
    float value = static_cast<float>(fo.values[i]) / WW_FEATURE_DIVISOR;
    int q = static_cast<int>(lroundf(value / impl->input_scale)) + impl->input_zero_point;
    if (q < -128) q = -128;
    if (q > 127)  q = 127;
    out[i] = static_cast<int8_t>(q);
  }
  return true;
}

bool WakeWordDetector::runInference(const int8_t features[WW_FEATURE_COUNT],
                                    float* probability) {
  WWImpl* impl = static_cast<WWImpl*>(impl_);

  // Модели v1 ждут [1,1,40], v2 — [1,3,40]. Оба варианта поддержаны ниже:
  // если входной тензор шире одного среза (v2), сдвигаем скользящее окно
  // и дописываем новый срез в конец; если ровно один срез (v1) — просто
  // перезаписываем. "Ирина" — v2, этот путь и используется.
  const size_t input_elems = impl->input->bytes;  // int8 -> байт == элементов
  if (input_elems < WW_FEATURE_COUNT) return false;
  if (input_elems > WW_FEATURE_COUNT) {
    // v2-модель: сдвигаем окно, новый срез кладём в конец.
    memmove(impl->input->data.int8,
            impl->input->data.int8 + WW_FEATURE_COUNT,
            input_elems - WW_FEATURE_COUNT);
    memcpy(impl->input->data.int8 + (input_elems - WW_FEATURE_COUNT),
           features, WW_FEATURE_COUNT);
  } else {
    memcpy(impl->input->data.int8, features, WW_FEATURE_COUNT);
  }

  if (impl->interpreter->Invoke() != kTfLiteOk) return false;

  // Выход обычно uint8 со scale 1/256; но читаем параметры из тензора.
  float p = 0.0f;
  if (impl->output->type == kTfLiteUInt8) {
    p = (impl->output->data.uint8[0] - impl->output->params.zero_point) *
        impl->output->params.scale;
  } else if (impl->output->type == kTfLiteInt8) {
    p = (impl->output->data.int8[0] - impl->output->params.zero_point) *
        impl->output->params.scale;
  } else if (impl->output->type == kTfLiteFloat32) {
    p = impl->output->data.f[0];
  } else {
    return false;
  }

  if (p < 0.0f) p = 0.0f;
  if (p > 1.0f) p = 1.0f;
  *probability = p;
  return true;
}

bool WakeWordDetector::process(const int16_t* samples, size_t count) {
  if (!ready_) return false;

  bool detected = false;
  size_t offset = 0;

  while (offset < count) {
    // Добиваем накопитель до полного шага (160 сэмплов).
    size_t space = WW_SAMPLES_PER_STRIDE - pending_count_;
    size_t take = std::min(space, count - offset);
    memcpy(pending_ + pending_count_, samples + offset, take * sizeof(int16_t));
    pending_count_ += take;
    offset += take;

    if (pending_count_ < WW_SAMPLES_PER_STRIDE) break;  // ждём ещё данных

    int8_t features[WW_FEATURE_COUNT];
    size_t consumed = 0;
    bool got = generateFeatures(pending_, pending_count_, &consumed, features);

    // Фронтенд мог съесть не всё — сдвигаем остаток.
    if (consumed > 0 && consumed <= pending_count_) {
      memmove(pending_, pending_ + consumed,
              (pending_count_ - consumed) * sizeof(int16_t));
      pending_count_ -= consumed;
    } else {
      pending_count_ = 0;
    }

    if (!got) continue;

    float p = 0.0f;
    if (!runInference(features, &p)) continue;
    last_probability_ = p;

    // Первые срезы после старта игнорируем: фронтенд ещё не оценил шум.
    if (ignore_slices_ > 0) { ignore_slices_--; continue; }

    prob_window_[prob_window_idx_] = p;
    prob_window_idx_ = (prob_window_idx_ + 1) % prob_window_size_;
    if (prob_window_filled_ < prob_window_size_) prob_window_filled_++;

    if (prob_window_filled_ < prob_window_size_) continue;

    float sum = 0.0f;
    for (uint8_t i = 0; i < prob_window_size_; i++) sum += prob_window_[i];
    float avg = sum / prob_window_size_;

    if (avg >= cutoff_) {
      printf("[WW] Детект '%s' (avg=%.3f)\n", name_, avg);
      detected = true;
      // Сбрасываем окно, иначе следующий срез снова превысит порог
      // и мы получим серию детектов на одном слове.
      memset(prob_window_, 0, sizeof(prob_window_));
      prob_window_filled_ = 0;
      prob_window_idx_ = 0;
      ignore_slices_ = WW_IGNORE_SLICES / 2;
    }
  }

  return detected;
}

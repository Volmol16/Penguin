#include "eyes.h"
#include "ssd1306.h"

#include "esp_timer.h"
#include "esp_random.h"
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Значения по умолчанию — приблизительный центр экрана 128x64 для двух
// глаз. ПОДГОНИТЕ под свои отверстия (через калибровку или вручную здесь).
static EyeConfig s_cfg = {
  .left_cx = 38, .left_cy = 32,
  .right_cx = 90, .right_cy = 32,
  .radius = 20,
  .pupil_radius = 5,
};

static Emotion s_emotion = Emotion::NEUTRAL;

// --- моргание: неблокирующая state machine ---
enum class BlinkPhase { IDLE, CLOSING, CLOSED, OPENING };
static BlinkPhase s_blink_phase = BlinkPhase::IDLE;
static int64_t s_blink_t0_us = 0;
static int64_t s_next_auto_blink_us = 0;

static const int64_t BLINK_CLOSE_US = 90000;
static const int64_t BLINK_HOLD_US  = 50000;
static const int64_t BLINK_OPEN_US  = 120000;

void eyes_init(const EyeConfig& cfg) {
  s_cfg = cfg;
  s_emotion = Emotion::NEUTRAL;
  s_blink_phase = BlinkPhase::IDLE;
  s_next_auto_blink_us = esp_timer_get_time() + 2000000;  // первое моргание через 2с
}

const EyeConfig& eyes_get_config() { return s_cfg; }
void eyes_set_config(const EyeConfig& cfg) { s_cfg = cfg; }

void eyes_set_emotion(Emotion e) { s_emotion = e; }

void eyes_blink() {
  if (s_blink_phase == BlinkPhase::IDLE) {
    s_blink_phase = BlinkPhase::CLOSING;
    s_blink_t0_us = esp_timer_get_time();
  }
}

// Возвращает 0.0 (глаз полностью открыт) .. 1.0 (полностью закрыт) для
// текущего момента анимации моргания, попутно переключая фазы.
static float blinkCoverage() {
  if (s_blink_phase == BlinkPhase::IDLE) return 0.0f;
  int64_t now = esp_timer_get_time();
  int64_t t = now - s_blink_t0_us;

  switch (s_blink_phase) {
    case BlinkPhase::CLOSING: {
      float frac = (float)t / (float)BLINK_CLOSE_US;
      if (frac >= 1.0f) { s_blink_phase = BlinkPhase::CLOSED; s_blink_t0_us = now; return 1.0f; }
      return frac;
    }
    case BlinkPhase::CLOSED: {
      if (t >= BLINK_HOLD_US) { s_blink_phase = BlinkPhase::OPENING; s_blink_t0_us = now; }
      return 1.0f;
    }
    case BlinkPhase::OPENING: {
      float frac = (float)t / (float)BLINK_OPEN_US;
      if (frac >= 1.0f) { s_blink_phase = BlinkPhase::IDLE; return 0.0f; }
      return 1.0f - frac;
    }
    default:
      return 0.0f;
  }
}

// Рисует один глаз по мотивам присланной картинки: ПУНКТИРНОЕ кольцо
// (имитация точечной/halftone-текстуры оригинала — на 128x64 1-бит
// настоящую точечную растушёвку не воспроизвести, дискретный пунктир даёт
// похожее ощущение) + маленький ромбик-блик, смещённый от центра к
// переносице (как на картинке у обоих глаз).
//
// inward: +1 если это ЛЕВЫЙ глаз (блик смещается вправо, т.е. к центру
// экрана), -1 если ПРАВЫЙ (блик смещается влево). Определяется по
// положению cx относительно середины экрана — менять не нужно.
static void drawEye(int cx, int cy, int r, int pupilR, Emotion emotion, float lidCoverage) {
  int lidPx = (int)(lidCoverage * r);
  int inward = (cx < OLED_WIDTH / 2) ? +1 : -1;
  int pupilCx = cx + inward * (r / 4);
  int pupilCy = cy - r / 3;

  for (int dy = -r; dy <= r; dy++) {
    int y = cy + dy;
    if (y < 0 || y >= OLED_HEIGHT) continue;

    // --- маска моргания: верхнее/нижнее веко сходятся к горизонтали глаза ---
    if (dy < -r + lidPx) continue;
    if (dy >  r - lidPx) continue;

    // --- маска эмоции: какая часть кольца по вертикали видна ---
    switch (emotion) {
      case Emotion::HAPPY:
        if (dy > 1) continue;              // видна верхняя половина — прищур "дугой"
        break;
      case Emotion::SAD:
        if (dy < -r / 3) continue;         // верх прикрыт — печальный взгляд вниз
        break;
      default:
        break;  // NEUTRAL / ANGRY / SURPRISED — вертикально не режем
    }

    int dxMax = (int)sqrtf((float)(r * r - dy * dy));
    for (int dx = -dxMax; dx <= dxMax; dx++) {
      int x = cx + dx;
      if (x < 0 || x >= OLED_WIDTH) continue;

      // ANGRY: срезаем верхний внутренний угол по диагонали (нахмуренная
      // бровь), зеркально для левого/правого глаза относительно переносицы.
      if (emotion == Emotion::ANGRY && dy < -r / 3) {
        bool innerSide = (cx < OLED_WIDTH / 2) ? (dx > 0) : (dx < 0);
        int cut = (int)((-dy - r / 3) * 1.4f);
        if (innerSide && dx > (dxMax - cut)) continue;
      }

      float dist = sqrtf((float)(dx * dx + dy * dy));
      bool inBand = fabsf(dist - r) <= 1.6f;   // толщина кольца ~3px
      if (!inBand) continue;

      // Пунктир по контуру: периодически гасим участки дуги — имитация
      // точечной текстуры оригинала. 14°/9° подобраны визуально (см.
      // превью), при желании подправьте соотношение для более частого/
      // редкого пунктира.
      float angleDeg = atan2f((float)dy, (float)dx) * 180.0f / (float)M_PI;
      if (angleDeg < 0) angleDeg += 360.0f;
      bool dashOn = fmodf(angleDeg, 14.0f) < 9.0f;
      if (dashOn) ssd1306_set_pixel(x, y, true);
    }
  }

  // Зрачок — маленький РОМБ (манхэттенское расстояние вместо круга),
  // ближе по силуэту к "крестику"-блику на картинке, чем сплошная точка.
  for (int dy = -pupilR; dy <= pupilR; dy++) {
    int y = pupilCy + dy;
    if (y < cy - r + lidPx || y > cy + r - lidPx) continue;
    if (emotion == Emotion::HAPPY && (y - cy) > 1) continue;
    if (emotion == Emotion::SAD && (y - cy) < -r / 3) continue;
    for (int dx = -pupilR; dx <= pupilR; dx++) {
      if (abs(dx) + abs(dy) <= pupilR) {
        ssd1306_set_pixel(pupilCx + dx, y, true);
      }
    }
  }
}

void eyes_update() {
  // Автоматическое моргание раз в 2-6 секунд (случайный интервал — иначе
  // выглядит механически). Можно убрать/заменить реальной логикой, когда
  // игрушка подключится к реальному диалогу (моргать перед ответом и т.п.).
  int64_t now = esp_timer_get_time();
  if (now >= s_next_auto_blink_us) {
    eyes_blink();
    s_next_auto_blink_us = now + (2000000 + (esp_random() % 4000000));
  }

  float lid = blinkCoverage();

  ssd1306_clear();

  int radius = s_cfg.radius;
  // 1.15, не больше: при радиусе по умолчанию (20px) и текущем расстоянии
  // между центрами глаз больший множитель сводит кольца почти вплотную
  // (проверено на рендере) — если раздвинете глаза шире при калибровке,
  // можно смело увеличить.
  if (s_emotion == Emotion::SURPRISED) radius = (int)(s_cfg.radius * 1.15f);

  drawEye(s_cfg.left_cx, s_cfg.left_cy, radius, s_cfg.pupil_radius, s_emotion, lid);
  drawEye(s_cfg.right_cx, s_cfg.right_cy, radius, s_cfg.pupil_radius, s_emotion, lid);

  ssd1306_flush();
}

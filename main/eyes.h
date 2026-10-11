/*
 * eyes.h — анимированные глазки для OLED 128x64 (кольцо-радужка + зрачок,
 * по мотивам присланной вами картинки, адаптировано под монохромный экран:
 * вместо "свечения" — контурное кольцо + маленькая точка-блик внутри).
 */
#ifndef EYES_H
#define EYES_H

#include <stdint.h>

enum class Emotion {
  NEUTRAL,
  HAPPY,
  SAD,
  ANGRY,
  SURPRISED,
};

// Координаты и размеры — то самое, что нужно подогнать под отверстия в
// корпусе. Можно менять прямо здесь (и перепрошивать), а можно подогнать
// live через калибровку в serial-мониторе (см. main.cpp, команды w/a/s/d
// и т.д.) и потом командой 'p' получить готовые числа для вставки сюда.
struct EyeConfig {
  int left_cx, left_cy;
  int right_cx, right_cy;
  int radius;        // радиус кольца-радужки
  int pupil_radius;   // радиус точки-зрачка внутри
};

void eyes_init(const EyeConfig& cfg);
const EyeConfig& eyes_get_config();
void eyes_set_config(const EyeConfig& cfg);

void eyes_set_emotion(Emotion e);

// Запускает одно моргание (неблокирующе — анимация считается в eyes_update()).
void eyes_blink();

// Вызывать в основном цикле на каждом кадре (например, раз в ~30мс).
// Сама чистит буфер, рисует оба глаза с учётом эмоции/моргания и
// отправляет кадр на экран (ssd1306_flush()).
void eyes_update();

#endif  // EYES_H

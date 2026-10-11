/*
 * ssd1306.h — минимальный драйвер OLED 0.96" 128x64 I2C (контроллер
 * SSD1306 — это стандартный чип почти во всех таких дисплеях; если у вас
 * окажется SH1106 — экран будет сдвинут на 2px, см. комментарий в .cpp).
 *
 * Написан вручную под ваш конкретный дисплей, а не взят как внешняя
 * компонента — меньше риска словить ошибку пути/версии, как уже было
 * с esp-tflite-micro/esp-micro-speech-features.
 */
#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define OLED_WIDTH   128
#define OLED_HEIGHT  64

#ifdef __cplusplus
extern "C" {
#endif

// Поднимает I2C-шину и сам дисплей. sda_pin/scl_pin — ваша реальная
// разводка, i2c_addr обычно 0x3C (реже 0x3D — смотрите на шильдике/в
// документации конкретной платы, либо прогоните I2C-сканер, если не
// уверены).
bool ssd1306_init(int sda_pin, int scl_pin, uint8_t i2c_addr);

// Очищает буфер в памяти ESP32 (НЕ сам экран — нужен ssd1306_flush()).
void ssd1306_clear(void);

// on=true зажигает пиксель, false — гасит. Без проверки границ на каждый
// пиксель ради скорости — вызывающий код должен сам следить за диапазоном
// (0..127 / 0..63), что и делает eyes.cpp.
void ssd1306_set_pixel(int x, int y, bool on);

void ssd1306_fill_rect(int x, int y, int w, int h, bool on);

// Моноширинный шрифт 5x7 (font5x7.h), scale=1 обычный размер,
// scale=2 — вдвое крупнее (для коротких заголовков).
void ssd1306_draw_char(int x, int y, char c, bool on, int scale);
void ssd1306_draw_string(int x, int y, const char* s, bool on, int scale);

// Отправляет буфер на экран по I2C. Вызывать один раз за кадр, после всех
// операций рисования — так дешевле, чем слать по пикселю.
void ssd1306_flush(void);

#ifdef __cplusplus
}
#endif

#endif  // SSD1306_H

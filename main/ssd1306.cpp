#include "ssd1306.h"
#include "font5x7.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "ssd1306";

// Буфер экрана: 128 столбцов x 8 страниц по 8 бит (стандартная page-адресация
// SSD1306). fb[page*128 + x], бит y%8 — светится ли пиксель (x, page*8+y%8).
static uint8_t s_fb[OLED_WIDTH * (OLED_HEIGHT / 8)];

static i2c_master_bus_handle_t s_bus = nullptr;
static i2c_master_dev_handle_t s_dev = nullptr;

static bool writeCommand(uint8_t cmd) {
  uint8_t buf[2] = {0x00, cmd};  // 0x00 = control byte "дальше идёт команда"
  return i2c_master_transmit(s_dev, buf, sizeof(buf), 1000) == ESP_OK;
}

static bool writeCommand2(uint8_t cmd, uint8_t arg) {
  uint8_t buf[3] = {0x00, cmd, arg};
  return i2c_master_transmit(s_dev, buf, sizeof(buf), 1000) == ESP_OK;
}

bool ssd1306_init(int sda_pin, int scl_pin, uint8_t i2c_addr) {
  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = I2C_NUM_0;
  bus_cfg.sda_io_num = (gpio_num_t)sda_pin;
  bus_cfg.scl_io_num = (gpio_num_t)scl_pin;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = 7;
  bus_cfg.flags.enable_internal_pullup = true;

  if (i2c_new_master_bus(&bus_cfg, &s_bus) != ESP_OK) {
    ESP_LOGE(TAG, "i2c_new_master_bus failed (проверьте SDA/SCL пины)");
    return false;
  }

  i2c_device_config_t dev_cfg = {};
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address = i2c_addr;
  dev_cfg.scl_speed_hz = 400000;

  if (i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev) != ESP_OK) {
    ESP_LOGE(TAG, "i2c_master_bus_add_device failed");
    return false;
  }

  // Стандартная последовательность инициализации SSD1306 128x64.
  // Если картинка окажется сдвинутой на пару пикселей или "зеркальной" —
  // это обычно SH1106 вместо SSD1306 (визуально похожие модули часто
  // путают при продаже); скажите мне, какой эффект видите, подправим.
  const uint8_t initSeq[] = {
    0xAE,        // display off
    0xD5, 0x80,  // clock divide
    0xA8, 0x3F,  // multiplex 64
    0xD3, 0x00,  // display offset 0
    0x40,        // start line 0
    0x8D, 0x14,  // charge pump enable
    0x20, 0x00,  // memory addressing mode: horizontal
    0xA1,        // segment remap (зеркально по X)
    0xC8,        // COM scan direction (зеркально по Y)
    0xDA, 0x12,  // COM pins hardware config
    0x81, 0xCF,  // contrast
    0xD9, 0xF1,  // pre-charge
    0xDB, 0x40,  // VCOMH deselect level
    0xA4,        // resume to RAM content (не "весь экран горит")
    0xA6,        // normal display (не инвертированный)
    0xAF,        // display ON
  };
  for (size_t i = 0; i < sizeof(initSeq); i++) {
    if (!writeCommand(initSeq[i])) {
      ESP_LOGE(TAG, "Команда инициализации 0x%02x не прошла — проверьте "
               "адрес на I2C (сейчас 0x%02x) и подключение", initSeq[i], i2c_addr);
      return false;
    }
  }

  ssd1306_clear();
  ssd1306_flush();
  ESP_LOGI(TAG, "OLED инициализирован (SDA=%d SCL=%d addr=0x%02x)",
           sda_pin, scl_pin, i2c_addr);
  return true;
}

void ssd1306_clear(void) {
  memset(s_fb, 0, sizeof(s_fb));
}

void ssd1306_set_pixel(int x, int y, bool on) {
  if ((unsigned)x >= OLED_WIDTH || (unsigned)y >= OLED_HEIGHT) return;
  int page = y / 8;
  uint8_t bit = 1 << (y % 8);
  uint8_t& b = s_fb[page * OLED_WIDTH + x];
  if (on) b |= bit; else b &= ~bit;
}

void ssd1306_fill_rect(int x, int y, int w, int h, bool on) {
  for (int yy = y; yy < y + h; yy++) {
    for (int xx = x; xx < x + w; xx++) {
      ssd1306_set_pixel(xx, yy, on);
    }
  }
}

void ssd1306_draw_char(int x, int y, char c, bool on, int scale) {
  if (c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR) c = ' ';
  const uint8_t* glyph = FONT5X7[c - FONT_FIRST_CHAR];
  for (int col = 0; col < 5; col++) {
    uint8_t colBits = glyph[col];
    for (int row = 0; row < 7; row++) {
      bool bitOn = (colBits >> row) & 0x01;
      if (!bitOn) continue;
      if (scale == 1) {
        ssd1306_set_pixel(x + col, y + row, on);
      } else {
        ssd1306_fill_rect(x + col * scale, y + row * scale, scale, scale, on);
      }
    }
  }
}

void ssd1306_draw_string(int x, int y, const char* s, bool on, int scale) {
  int cx = x;
  int advance = (5 + 1) * (scale < 1 ? 1 : scale);
  while (*s) {
    ssd1306_draw_char(cx, y, *s, on, scale);
    cx += advance;
    s++;
  }
}

void ssd1306_flush(void) {
  // Горизонтальная адресация: один раз задаём диапазон столбцов/страниц,
  // дальше контроллер сам переходит на следующую страницу по заполнении —
  // можно слать буфер одним потоком, не выставляя адрес на каждую страницу.
  writeCommand2(0x21, 0);              // column 0
  writeCommand(127);
  writeCommand2(0x22, 0);              // page 0
  writeCommand(7);

  // Шлём буфер кусками: контроль-байт 0x40 (данные) + N байт пикселей.
  // Кусками, а не одним 1025-байтным пакетом — надёжнее на некоторых
  // портах I2C-драйвера и проще отлаживать при проблемах.
  const size_t CHUNK = 128;
  uint8_t packet[1 + CHUNK];
  for (size_t off = 0; off < sizeof(s_fb); off += CHUNK) {
    size_t n = CHUNK;
    packet[0] = 0x40;
    memcpy(packet + 1, s_fb + off, n);
    i2c_master_transmit(s_dev, packet, 1 + n, 1000);
  }
}

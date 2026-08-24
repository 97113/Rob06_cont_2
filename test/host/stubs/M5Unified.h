// ---------------------------------------------------------------------------
// Host stub of M5Unified. Enough surface to COMPILE ui.cpp, main.cpp and
// can_scan.cpp - it draws nothing and reports no touches. The point is that a
// typo in the UI fails on a laptop in a second instead of on the bench.
// ---------------------------------------------------------------------------
#pragma once
#include <Arduino.h>
#include <stdint.h>

namespace fonts { struct Font { int dummy; }; extern Font Font2, Font4; }

enum textdatum_t {
  top_left = 0, top_center, top_right,
  middle_left, middle_center, middle_right,
  bottom_left, bottom_center, bottom_right,
};

#define TFT_BLACK       0x0000
#define TFT_WHITE       0xFFFF
#define TFT_RED         0xF800
#define TFT_GREEN       0x07E0
#define TFT_BLUE        0x001F
#define TFT_CYAN        0x07FF
#define TFT_YELLOW      0xFFE0
#define TFT_ORANGE      0xFD20
#define TFT_NAVY        0x000F
#define TFT_MAROON      0x7800
#define TFT_PURPLE      0x780F
#define TFT_OLIVE       0x7BE0
#define TFT_DARKGREY    0x7BEF
#define TFT_LIGHTGREY   0xD69A
#define TFT_DARKGREEN   0x03E0
#define TFT_DARKCYAN    0x03EF
#define TFT_GREENYELLOW 0xB7E0

struct M5Display {
  void setRotation(int) {}
  void fillScreen(uint16_t) {}
  void fillRect(int, int, int, int, uint16_t) {}
  void drawRect(int, int, int, int, uint16_t) {}
  void fillRoundRect(int, int, int, int, int, uint16_t) {}
  void drawRoundRect(int, int, int, int, int, uint16_t) {}
  void drawLine(int, int, int, int, uint16_t) {}
  void setFont(const fonts::Font*) {}
  void setTextColor(uint16_t) {}
  void setTextColor(uint16_t, uint16_t) {}
  void setTextDatum(textdatum_t) {}
  void drawString(const char*, int, int) {}
};

struct TouchDetail {
  int16_t x = 0, y = 0;
  bool wasReleased() const { return false; }
  int  distanceX()   const { return 0; }
  int  distanceY()   const { return 0; }
};

struct M5Touch { TouchDetail getDetail() const { return TouchDetail{}; } };

struct M5Button {
  bool wasPressed() const { return false; }
  bool isPressed()  const { return false; }
};

struct M5Config { bool internal_imu = true, internal_mic = true, internal_spk = true; };

struct M5Unified {
  M5Display Display;
  M5Touch   Touch;
  M5Button  BtnA, BtnB, BtnC;
  M5Config  config() { return M5Config{}; }
  void begin(M5Config&) {}
  void update() {}
};

extern M5Unified M5;

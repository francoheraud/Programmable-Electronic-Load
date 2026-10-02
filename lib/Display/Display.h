#pragma once
#include <Arduino.h>
#include <Wire.h>

#define OLED_I2C_ADDR  0x3C
#define OLED_WIDTH     128
#define OLED_HEIGHT    64

extern TwoWire Wire2;

class Display {
public:
  bool begin();
  void clear();

  void showStatus(int state, float measV, float measI, float setV, float setI,
                   const float temps[4], uint32_t secondsRemaining);

  void showMessage(const char* line1, const char* line2 = nullptr);

  static constexpr uint8_t ROWS = 8;   // 64px / 8px text rows
  static constexpr uint8_t COLS = 21;  // 128px / 6px glyphs

  void beginFrame();
  void drawLine(uint8_t row, const String& text, bool highlighted = false);
  void endFrame();

private:
  bool _ready = false;
};

extern Display display;

#pragma once
// Recording stand-in for lib/Display, native tests only.
#include <Arduino.h>
#include <vector>

class Display {
public:
  std::vector<std::string> rows = std::vector<std::string>(8);
  int frames = 0;

  void beginFrame() { for (auto& r : rows) r.clear(); }
  void drawLine(uint8_t row, const String& text, bool = false) { if (row < rows.size()) rows[row] = text; }
  void endFrame() { frames++; }
  void showMessage(const char*, const char* = nullptr) {}
};

inline Display display;

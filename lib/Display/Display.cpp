#include "Display.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

TwoWire Wire2 = TwoWire(0);
static Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire2, -1);

Display display;

bool Display::begin() {
  Wire2.beginTransmission(OLED_I2C_ADDR);
  const bool acknowledged = Wire2.endTransmission() == 0;
  // Wire2 was configured by setup(); retain its PCB pins.
  _ready = acknowledged && oled.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR, true, false);
  if (!_ready) {
    Serial.println("Display: SSD1306 did not ACK on I2C_2 -- check "
                    "OLED_I2C_ADDR, wiring, and the module's BS1/BS0 strap.");
    return false;
  }

  oled.setRotation(2); // Enclosure mounts the PCB upside down: rotate all screens 180 degrees.
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setTextWrap(false);
  oled.display();
  return true;
}

void Display::clear() {
  if (!_ready) return;
  oled.clearDisplay();
}

void Display::showStatus(int state, float measV, float measI, float setV, float setI,
                          const float temps[4], uint32_t secondsRemaining) {
  if (!_ready) return;

  static const char* stateNames[] = {"INIT", "IDLE", "RUN", "FAULT", "DONE"};
  const char* stateName = (state >= 0 && state <= 4) ? stateNames[state] : "?";

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);

  oled.printf("State: %s\n", stateName);
  oled.printf("V: %5.2fV (set %5.2f)\n", measV, setV);
  oled.printf("I: %5.2fA (set %5.2f)\n", measI, setI);
  oled.printf("T1-4: %.0f %.0f %.0f %.0fC\n", temps[0], temps[1], temps[2], temps[3]);

  uint32_t mins = secondsRemaining / 60;
  uint32_t secs = secondsRemaining % 60;
  oled.printf("Time left: %02u:%02u\n", mins, secs);

  oled.display();
}

void Display::beginFrame() {
  if (!_ready) return;
  oled.clearDisplay();
}

void Display::drawLine(uint8_t row, const String& text, bool highlighted) {
  if (!_ready) return;
  if (row >= ROWS) return;

  uint16_t y = row * 8;
  if (highlighted) {
    oled.fillRect(0, y, OLED_WIDTH, 8, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
  } else {
    oled.setTextColor(SSD1306_WHITE);
  }

  oled.setTextSize(1);
  oled.setCursor(0, y);
  oled.print(text.substring(0, COLS));
}

void Display::endFrame() {
  if (!_ready) return;
  oled.display();
}

void Display::showMessage(const char* line1, const char* line2) {
  if (!_ready) return;

  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setCursor(0, 10);
  oled.println(line1);

  if (line2) {
    oled.setTextSize(1);
    oled.setCursor(0, 40);
    oled.println(line2);
  }

  oled.display();
}

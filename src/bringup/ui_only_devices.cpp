#include "devices.h"
#include <Display.h>

#ifdef UI_ONLY_BRINGUP
// This build intentionally contains no ADC, DAC, fan or load-control driver.
bool devicesCommand(int argc, String argv[]) {
  if (argc == 0 || argv[0] != "scan") return false;
  Serial.println("Scanning OLED bus (SDA 17, SCL 18)...");
  unsigned found = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire2.beginTransmission(address);
    if (Wire2.endTransmission() == 0) {
      Serial.printf("I2C ACK at 0x%02X\n", address);
      ++found;
    }
  }
  Serial.printf("%u device(s) found; OLED configured at 0x%02X\n", found, OLED_I2C_ADDR);
  return true;
}
void devicesTick(uint32_t) {}
void devicesSafeState() {}
void devicesSense() { Serial.println("UI-only build: sensor readings unavailable."); }
void devicesPrintHelp() {
  Serial.println("  scan      scan OLED I2C bus\nUI-only build: load/sensor commands excluded; UI readings are simulated.");
}
#endif

#include "devices.h"
#include <Wire.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <Pins.h>
#include <DeviceConfig.h>
#include <SystemState.h>
#include <Display.h>
#include <INA.h>
#include <ADS7128.h>
#include <ADS1015.h>
#include <DAC.h>
#include <SdLog.h>

// ---- safety limits for DAC writes (edit here if you need more) ----
static const float    MAX_BRINGUP_LEG_A = 1.0f;   // per-leg current setpoint cap
static const uint32_t DEFAULT_HOLD_S    = 10;     // setpoints return to 0 after this long
static const uint32_t MAX_HOLD_S        = 120;
static const float    DAC_FS_VOLTS      = 2.5f;   // full-scale set by DAC.cpp (REFDIV_EN + gain 1)

// Trip-threshold scaling from the schematic
static const float OV_DIV        = 16.0f;    // Prog_OV = Load+ / 16 (R65 15k / R63 1k), DAC OUT7 drives it directly
static const float OC_DAC_DIV    = 16.0f;    // Prog_OC = DAC OUT6 / 16 (R50 15k / R52 1k)
static const float SHUNT_OHMS    = 0.025f;   // per-leg shunt; comparator sees the average of the 6 shunt voltages
static const float SYSTEM_OV_V   = 33.3f;    // fixed hardware limit (U12A)
static const float SYSTEM_OC_A   = 33.5f;    // fixed hardware limit, total current (U11B, 0.1395 V on the shunt average)

static const uint8_t INA_REG_MFR_ID = 0x3E;       // reads 0x5449 ("TI") on TI parts

static SPIClass sdSpi(HSPI);
static bool wire1Started = false;
static bool inaConfigured = false;
static bool ads7128Ready = false;
static bool dacStarted = false;
static bool sdMounted = false;
static bool fanStarted = false;
static bool armed = false;
static uint16_t dacCodes[DAC_NUM_SETPOINTS] = {0, 0, 0, 0, 0, 0};
static bool dacHolding = false;
static uint32_t dacHoldUntilMs = 0;

// ---------------- helpers ----------------

static bool parseHex(const String& s, uint32_t& out) {
  if (s.length() == 0) return false;
  char* end = nullptr;
  unsigned long v = strtoul(s.c_str(), &end, 16);
  if (*end != '\0') return false;
  out = (uint32_t)v;
  return true;
}

static bool parseNum(const String& s, float& out) {
  if (s.length() == 0) return false;
  char* end = nullptr;
  float v = strtof(s.c_str(), &end);
  if (*end != '\0') return false;
  out = v;
  return true;
}

static void ensureWire1() {
  if (wire1Started) return;
  Wire1.begin(PIN_SDA_1, PIN_SCL_1);
  Wire1.setTimeOut(I2C_TRANSACTION_TIMEOUT_MS);
  wire1Started = true;
}

static TwoWire* busFor(uint32_t bus) {
  if (bus == 1) { ensureWire1(); return &Wire1; }
  if (bus == 2) { return &Wire2; }
  return nullptr;
}

static const char* i2cErr(uint8_t st) {
  switch (st) {
    case 1: return "data too long";
    case 2: return "NACK on address (no device there)";
    case 3: return "NACK on data";
    case 4: return "other error";
    case 5: return "timeout";
    default: return "?";
  }
}

static bool i2cReadReg(TwoWire& w, uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n) {
  w.beginTransmission(addr);
  w.write(reg);
  uint8_t st = w.endTransmission(false);
  if (st != 0) {
    Serial.printf("  0x%02X: address phase failed (%u: %s)\n", addr, st, i2cErr(st));
    return false;
  }
  uint8_t got = w.requestFrom(addr, n);
  if (got != n) {
    Serial.printf("  0x%02X: read returned %u of %u bytes\n", addr, got, n);
    return false;
  }
  for (uint8_t i = 0; i < n; i++) { buf[i] = w.read(); }
  return true;
}

// ---------------- scan / raw register access ----------------

static const char* knownDevice(uint8_t a) {
  if (a >= 0x40 && a <= 0x45) return "INA";
  if (a == 0x10) return "ADS7128";
  if (a == 0x3C) return "OLED";
  if (a == 0x48) return "ADS1015";
  return "unknown";
}

static void scanBus(const char* label, TwoWire& w, const uint8_t* expected, uint8_t nExpected) {
  Serial.printf("%s:\n", label);
  bool seen[127] = {false};
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    w.beginTransmission(addr);
    if (w.endTransmission() == 0) {
      seen[addr] = true;
      found++;
      Serial.printf("  0x%02X  %s\n", addr, knownDevice(addr));
    }
  }
  if (found == 0) Serial.println("  (nothing responded)");
  for (uint8_t i = 0; i < nExpected; i++) {
    if (!seen[expected[i]]) Serial.printf("  MISSING 0x%02X (%s)\n", expected[i], knownDevice(expected[i]));
  }
}

static void cmdScan() {
  ensureWire1();
  static const uint8_t exp1[] = {0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x10};
  static const uint8_t exp2[] = {0x3C, 0x48};
  scanBus("Wire1 (SDA=8 SCL=9: INAs, ADS7128)", Wire1, exp1, sizeof(exp1));
  scanBus("Wire2 (SDA=17 SCL=18: OLED, ADS1015)", Wire2, exp2, sizeof(exp2));
}

static void cmdRd(int argc, String argv[]) {
  uint32_t bus, addr, reg, n = 2;
  if (argc < 4 || !parseHex(argv[1], bus) || !parseHex(argv[2], addr) || !parseHex(argv[3], reg) ||
      (argc >= 5 && !parseHex(argv[4], n)) || n < 1 || n > 8) {
    Serial.println("usage: rd <bus 1|2> <addr hex> <reg hex> [bytes 1-8, default 2]");
    return;
  }
  TwoWire* w = busFor(bus);
  if (!w) { Serial.println("bus must be 1 or 2"); return; }
  uint8_t buf[8];
  if (!i2cReadReg(*w, (uint8_t)addr, (uint8_t)reg, buf, (uint8_t)n)) return;
  Serial.printf("bus%u 0x%02X reg 0x%02X =", (unsigned)bus, (unsigned)addr, (unsigned)reg);
  for (uint8_t i = 0; i < n; i++) Serial.printf(" %02X", buf[i]);
  Serial.println();
}

static void cmdWr(int argc, String argv[]) {
  uint32_t bus, addr, reg, b1, b2 = 0;
  if (argc < 5 || !parseHex(argv[1], bus) || !parseHex(argv[2], addr) || !parseHex(argv[3], reg) ||
      !parseHex(argv[4], b1) || b1 > 0xFF || (argc >= 6 && (!parseHex(argv[5], b2) || b2 > 0xFF))) {
    Serial.println("usage: wr <bus 1|2> <addr hex> <reg hex> <byte hex> [byte2 hex]  (bytes sent MSB first)");
    return;
  }
  TwoWire* w = busFor(bus);
  if (!w) { Serial.println("bus must be 1 or 2"); return; }
  w->beginTransmission((uint8_t)addr);
  w->write((uint8_t)reg);
  w->write((uint8_t)b1);
  if (argc >= 6) w->write((uint8_t)b2);
  uint8_t st = w->endTransmission();
  if (st == 0) Serial.println("write OK");
  else Serial.printf("write failed (%u: %s)\n", st, i2cErr(st));
}

// ---------------- INA (6 current monitors on Wire1) ----------------

static void configureIna(bool force) {
  ensureWire1();
  if (inaConfigured && !force) return;
  inaConfigured = ina.begin(inaAddrs, shuntCal, VBUS_LEG);
  Serial.printf("INA configuration (shunt-cal + ADC config on all 6): %s\n", inaConfigured ? "OK" : "FAILED");
}

static void cmdIna(int argc, String argv[]) {
  configureIna(argc >= 2 && argv[1] == "init");
  float vbus = 0.0f;
  bool vok = ina.readBusVoltage(inaAddrs[VBUS_LEG], vbus);
  if (vok) Serial.printf("VBUS (leg %u, 0x%02X) = %.3f V\n", VBUS_LEG + 1, inaAddrs[VBUS_LEG], vbus);
  else     Serial.printf("VBUS read FAILED (leg %u, 0x%02X)\n", VBUS_LEG + 1, inaAddrs[VBUS_LEG]);

  float total = 0.0f;
  for (uint8_t i = 0; i < 6; i++) {
    float cur = 0.0f;
    bool cok = ina.readCurrent(inaAddrs[i], cur);
    uint8_t id[2] = {0, 0};
    bool idok = i2cReadReg(Wire1, inaAddrs[i], INA_REG_MFR_ID, id, 2);
    if (cok) total += cur;
    Serial.printf("  leg%u 0x%02X  I=", i + 1, inaAddrs[i]);
    if (cok) Serial.printf("%+.4f A", cur); else Serial.print("FAILED  ");
    if (idok) Serial.printf("   reg0x3E=0x%02X%02X (TI parts read 5449)\n", id[0], id[1]);
    else Serial.println("   reg0x3E unreadable");
  }
  Serial.printf("  total I = %.4f A   P = %.3f W\n", total, vbus * total);
}

// ---------------- ADS7128 heatsink NTCs (Wire1) and ADS1015 probes (Wire2) ----------------

static void ensureAds7128(bool force) {
  ensureWire1();
  if (ads7128Ready && !force) return;
  ads7128Ready = ads7128.begin();
  Serial.printf("ADS7128 configuration: %s\n", ads7128Ready ? "OK" : "FAILED");
}

static void cmdNtc(int argc, String argv[]) {
  ensureAds7128(argc >= 2 && argv[1] == "init");
  uint16_t raw[6] = {0};
  bool rawOk[6];
  for (uint8_t ch = 0; ch < 6; ch++) rawOk[ch] = ads7128.readChannel(ch, raw[ch]);
  float t[6] = {0};
  bool tempsOk = ads7128.readTemperatures(t);
  for (uint8_t ch = 0; ch < 6; ch++) {
    Serial.printf("  NTC%u  ", ch + 1);
    if (rawOk[ch]) Serial.printf("raw=%5u (%.1f%% of FS)  T=%.1f C\n", raw[ch], 100.0f * raw[ch] / 65535.0f, t[ch]);
    else Serial.println("read FAILED");
  }
  Serial.printf("  readTemperatures(): %s\n", tempsOk ? "all OK" : "some channels failed");
}

static void cmdExt() {
  float t[4] = {0};
  bool ok = ads1015.readTemperatures(t);
  Serial.println("ADS1015 external probes (driver assumes all 4 are attached):");
  for (uint8_t i = 0; i < 4; i++) Serial.printf("  probe%u  T=%.1f C\n", i + 1, t[i]);
  Serial.printf("  readTemperatures(): %s\n", ok ? "all OK" : "some channels failed");
}

// ADS7128 register access (opcode-based protocol, address 0x10 on Wire1)
static void cmdA7(int argc, String argv[]) {
  uint32_t reg, val = 0;
  bool isRead = argc >= 3 && argv[1] == "r";
  bool isWrite = argc >= 4 && argv[1] == "w";
  if (!(isRead || isWrite) || !parseHex(argv[2], reg) || reg > 0xFF || (isWrite && (!parseHex(argv[3], val) || val > 0xFF))) {
    Serial.println("usage: a7 r <reg hex>   |   a7 w <reg hex> <value hex>");
    return;
  }
  ensureWire1();
  if (isRead) {
    Wire1.beginTransmission(ADS7128_I2C_ADDR);
    Wire1.write(ADS7128_OP_SINGLE_READ);
    Wire1.write((uint8_t)reg);
    uint8_t st = Wire1.endTransmission(false);
    if (st != 0) { Serial.printf("address phase failed (%u: %s)\n", st, i2cErr(st)); return; }
    if (Wire1.requestFrom((uint8_t)ADS7128_I2C_ADDR, (uint8_t)1) != 1) { Serial.println("no data returned"); return; }
    Serial.printf("ADS7128 reg 0x%02X = 0x%02X\n", (unsigned)reg, Wire1.read());
  } else {
    Wire1.beginTransmission(ADS7128_I2C_ADDR);
    Wire1.write(ADS7128_OP_SINGLE_WRITE);
    Wire1.write((uint8_t)reg);
    Wire1.write((uint8_t)val);
    uint8_t st = Wire1.endTransmission();
    if (st == 0) Serial.println("write OK");
    else Serial.printf("write failed (%u: %s)\n", st, i2cErr(st));
  }
}

void devicesSense() {
  ensureWire1();
  configureIna(false);
  ensureAds7128(false);

  float vbus = 0.0f, total = 0.0f, cur[6] = {0};
  bool ok = ina.readBusVoltage(inaAddrs[VBUS_LEG], vbus);
  for (uint8_t i = 0; i < 6; i++) {
    ok &= ina.readCurrent(inaAddrs[i], cur[i]);
    total += cur[i];
  }
  float ntc[6] = {0}, ext[4] = {0};
  ok &= ads7128.readTemperatures(ntc);
  ok &= ads1015.readTemperatures(ext);

  Serial.printf("V=%.3f  I=%.4f  P=%.3f | legs %.3f %.3f %.3f %.3f %.3f %.3f\n",
                vbus, total, vbus * total, cur[0], cur[1], cur[2], cur[3], cur[4], cur[5]);
  Serial.printf("NTC %.1f %.1f %.1f %.1f %.1f %.1f | EXT %.1f %.1f %.1f %.1f | %s\n",
                ntc[0], ntc[1], ntc[2], ntc[3], ntc[4], ntc[5], ext[0], ext[1], ext[2], ext[3],
                ok ? "all reads OK" : "SOME READS FAILED");
}

// ---------------- DAC (write-only SPI; verify with a multimeter) ----------------

static uint16_t ampsToCode(float a) {
  if (a < 0.0f) a = 0.0f;
  if (a > MAX_CURRENT_PER_LEG_A) a = MAX_CURRENT_PER_LEG_A;
  return (uint16_t)((a / MAX_CURRENT_PER_LEG_A) * DAC_FULL_SCALE_CODE);
}

static void dacZero() {
  for (uint8_t i = 0; i < DAC_NUM_SETPOINTS; i++) dacCodes[i] = 0;
  dacHolding = false;
  if (dacStarted) dac.writeAllSetpoints(dacCodes);
}

static void ensureDac() {
  if (dacStarted) return;
  dac.begin(SPI, PIN_SCK_2, PIN_MOSI_2, PIN_CS_2);
  dacStarted = true;
  dacZero();
  Serial.println("DAC started (SPI SCK=5 MOSI=4 CS=3), all setpoints written to 0");
}

static bool parseLeg(const String& s, int& first, int& last) {
  if (s == "all") { first = 0; last = DAC_NUM_SETPOINTS - 1; return true; }
  long n = s.toInt();
  if (n >= 1 && n <= DAC_NUM_SETPOINTS && s == String(n)) { first = last = (int)n - 1; return true; }
  return false;
}

static void dacApply(int first, int last, uint16_t code, uint32_t holdS) {
  for (int i = first; i <= last; i++) dacCodes[i] = code;
  dac.writeAllSetpoints(dacCodes);
  dacHolding = false;
  for (uint8_t i = 0; i < DAC_NUM_SETPOINTS; i++) { if (dacCodes[i] != 0) dacHolding = true; }
  dacHoldUntilMs = millis() + holdS * 1000UL;
  Serial.printf("DAC legs %d-%d code=0x%04X -> expect about %.4f V on the setpoint pin(s) (if 2.5 V full scale)\n",
                first + 1, last + 1, code, (float)code / 65535.0f * DAC_FS_VOLTS);
  if (dacHolding) Serial.printf("holding for %lu s, then back to 0 (or 'dac zero' / 'stop')\n", (unsigned long)holdS);
}

static void cmdDac(int argc, String argv[]) {
  if (argc >= 2 && argv[1] == "zero") {
    dacZero();
    Serial.println("all DAC setpoints = 0");
    return;
  }
  bool isSet = argc >= 4 && argv[1] == "set";
  bool isCode = argc >= 4 && argv[1] == "code";
  bool isOvOc = argc >= 3 && (argv[1] == "ov" || argv[1] == "oc");
  bool isOvV = argc >= 3 && argv[1] == "ovv";
  bool isOcA = argc >= 3 && argv[1] == "oca";
  if (!(isSet || isCode || isOvOc || isOvV || isOcA)) {
    Serial.println("usage: dac zero\n"
                   "       dac set <leg 1-6|all> <amps> [seconds]    (needs 'arm'; amps <= 1.0 per leg)\n"
                   "       dac code <leg 1-6|all> <code hex> [secs]  (needs 'arm'; same cap)\n"
                   "       dac ov <code hex> | dac oc <code hex>     (needs 'arm'; raw trip-threshold codes)\n"
                   "       dac ovv <load volts>                      (needs 'arm'; OV trip, max 40 V)\n"
                   "       dac oca <total amps>                      (needs 'arm'; OC trip, max 37.5 A)");
    return;
  }
  if (!armed) { Serial.println("refused: type 'arm' first (this drives the load stage)"); return; }
  ensureDac();

  if (isOvV || isOcA) {
    float x;
    if (!parseNum(argv[2], x) || x < 0) { Serial.println("value must be a number >= 0"); return; }
    float pinV;  // voltage on the comparator reference, then DAC output voltage
    if (isOvV) {
      pinV = x / OV_DIV;
      if (pinV > DAC_FS_VOLTS) { Serial.printf("refused: above DAC range (max %.1f V)\n", DAC_FS_VOLTS * OV_DIV); return; }
      if (x > SYSTEM_OV_V) Serial.printf("note: above the fixed %.1f V system OV, which trips first\n", SYSTEM_OV_V);
    } else {
      pinV = (x / 6.0f) * SHUNT_OHMS;
      float dacV = pinV * OC_DAC_DIV;
      if (dacV > DAC_FS_VOLTS) { Serial.printf("refused: above DAC range (max %.1f A)\n", DAC_FS_VOLTS / OC_DAC_DIV / SHUNT_OHMS * 6.0f); return; }
      pinV = dacV;
      if (x > SYSTEM_OC_A) Serial.printf("note: above the fixed %.1f A system OC, which trips first\n", SYSTEM_OC_A);
    }
    uint16_t code = (uint16_t)lroundf(pinV / DAC_FS_VOLTS * DAC_FULL_SCALE_CODE);
    if (isOvV) dac.writeOV(code); else dac.writeOC(code);
    Serial.printf("%s threshold %.3f %s -> code 0x%04X (DAC output %.4f V)\n",
                  isOvV ? "OV" : "OC", x, isOvV ? "V" : "A (total)", code, (float)code / 65535.0f * DAC_FS_VOLTS);
    return;
  }

  if (isOvOc) {
    uint32_t code;
    if (!parseHex(argv[2], code) || code > 0xFFFF) { Serial.println("code must be hex 0-FFFF"); return; }
    if (argv[1] == "ov") dac.writeOV((uint16_t)code); else dac.writeOC((uint16_t)code);
    Serial.printf("wrote %s register = 0x%04X\n", argv[1].c_str(), (unsigned)code);
    return;
  }

  int first, last;
  if (!parseLeg(argv[2], first, last)) { Serial.println("leg must be 1-6 or 'all'"); return; }
  uint32_t holdS = DEFAULT_HOLD_S;
  float holdArg;
  if (argc >= 5) {
    if (!parseNum(argv[4], holdArg) || holdArg < 1 || holdArg > MAX_HOLD_S) {
      Serial.printf("seconds must be 1-%lu\n", (unsigned long)MAX_HOLD_S);
      return;
    }
    holdS = (uint32_t)holdArg;
  }

  uint16_t code;
  if (isSet) {
    float amps;
    if (!parseNum(argv[3], amps) || amps < 0.0f) { Serial.println("amps must be a number >= 0"); return; }
    if (amps > MAX_BRINGUP_LEG_A) { Serial.printf("refused: above the %.2f A per-leg bring-up cap\n", MAX_BRINGUP_LEG_A); return; }
    code = ampsToCode(amps);
  } else {
    uint32_t c;
    if (!parseHex(argv[3], c) || c > 0xFFFF) { Serial.println("code must be hex 0-FFFF"); return; }
    if (c > ampsToCode(MAX_BRINGUP_LEG_A)) { Serial.printf("refused: above the cap (max 0x%04X)\n", ampsToCode(MAX_BRINGUP_LEG_A)); return; }
    code = (uint16_t)c;
  }
  dacApply(first, last, code, holdS);
}

// ---------------- fan, fault lines ----------------

static void cmdFan(int argc, String argv[]) {
  float pct;
  if (argc < 2 || !parseNum(argv[1], pct) || pct < 0 || pct > 100) { Serial.println("usage: fan <0-100 percent>"); return; }
  if (!fanStarted) {
    ledcSetup(0, 25000, 8);
    ledcAttachPin(PIN_PWM, 0);
    fanStarted = true;
  }
  ledcWrite(0, (uint8_t)(pct * 255.0f / 100.0f + 0.5f));
  Serial.printf("fan PWM (GPIO%d) = %.0f%%\n", PIN_PWM, pct);
}

static void printFault() {
  Serial.printf("HARD_DISABLE (GPIO%d) reads %d  (HIGH = hardware protection tripped)\n",
                PIN_HARD_DISABLE, digitalRead(PIN_HARD_DISABLE));
}

static void cmdFault(const String& cmd, int argc, String argv[]) {
  if (cmd == "fault") { printFault(); return; }
  if (cmd == "faultset") {
    if (argc < 2 || (argv[1] != "0" && argv[1] != "1")) { Serial.println("usage: faultset <0|1>   (1 = assert MCU_DISABLE, load off)"); return; }
    pinMode(PIN_FAULT_SET, OUTPUT);
    digitalWrite(PIN_FAULT_SET, argv[1] == "1" ? HIGH : LOW);
    Serial.printf("MCU_DISABLE (GPIO%d) = %s\n", PIN_FAULT_SET, argv[1] == "1" ? "HIGH (load disabled)" : "LOW");
    delay(5);
    printFault();
    return;
  }
  // faultrst: same pulse main.cpp uses to clear the hardware latch
  pinMode(PIN_FAULT_RST, OUTPUT);
  digitalWrite(PIN_FAULT_RST, HIGH);
  delay(10);
  digitalWrite(PIN_FAULT_RST, LOW);
  Serial.printf("pulsed MCU_RESET (GPIO%d) for 10 ms\n", PIN_FAULT_RST);
  printFault();
}

// ---------------- SD card ----------------

static bool ensureSd() {
  if (sdMounted) return true;
  sdMounted = sdLog.begin(sdSpi, PIN_SCK_1, PIN_MISO_1, PIN_MOSI_1, PIN_CS_1);
  return sdMounted;
}

static void cmdSd(int argc, String argv[]) {
  String sub = argc >= 2 ? argv[1] : String("init");
  if (!ensureSd()) return;

  if (sub == "init") {
    Serial.printf("SD mounted, %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));
  } else if (sub == "test") {
    const char* path = "/bringup.txt";
    const String payload = "bringup sd test 0123456789";
    if (SD.exists(path)) SD.remove(path);
    uint32_t t0 = millis();
    File f = SD.open(path, FILE_WRITE);
    if (!f) { Serial.println("FAIL: could not open file for writing"); return; }
    f.print(payload);
    f.close();
    f = SD.open(path, FILE_READ);
    if (!f) { Serial.println("FAIL: could not reopen file"); return; }
    String back = f.readString();
    f.close();
    SD.remove(path);
    Serial.printf("%s: wrote and read back %u bytes in %lu ms\n", back == payload ? "PASS" : "FAIL (data mismatch)",
                  (unsigned)payload.length(), (unsigned long)(millis() - t0));
  } else if (sub == "log") {
    if (!sdLog.startTest()) { Serial.println("FAIL: sdLog.startTest()"); return; }
    LogRecord r = {};
    r.state = LoadState::IDLE;
    for (uint8_t n = 0; n < 3; n++) {
      r.timestampMs = n * 500;
      r.voltage = 12.0f + n;
      r.current = 1.0f + n;
      r.power = r.voltage * r.current;
      for (uint8_t i = 0; i < 6; i++) { r.legCurrent[i] = r.current / 6.0f; r.tempInt[i] = 28.0f + i; }
      for (uint8_t i = 0; i < 4; i++) { r.tempExt[i] = 25.0f + i; }
      if (!sdLog.append(r)) { Serial.println("FAIL: sdLog.append()"); break; }
    }
    sdLog.endTest();
    Serial.println("wrote 3 sample rows with the real logger (see the file name above); 'sd ls' to list");
  } else if (sub == "ls") {
    File root = SD.open("/");
    if (!root) { Serial.println("FAIL: could not open root"); return; }
    File e = root.openNextFile();
    while (e) {
      Serial.printf("  %s%s  %u bytes\n", e.name(), e.isDirectory() ? "/" : "", (unsigned)e.size());
      e = root.openNextFile();
    }
    root.close();
  } else {
    Serial.println("usage: sd [init] | sd test | sd log | sd ls");
  }
}

// ---------------- dispatch ----------------

void devicesSafeState() {
  dacZero();
  armed = false;
}

void devicesTick(uint32_t now) {
  if (dacHolding && (int32_t)(now - dacHoldUntilMs) >= 0) {
    dacZero();
    Serial.println("DAC hold expired: all setpoints back to 0");
  }
}

void devicesPrintHelp() {
  Serial.println(
    "Device commands (hex args have no 0x needed):\n"
    "  scan                      I2C scan of both buses, flags missing devices\n"
    "  rd <bus> <addr> <reg> [n] read n bytes (default 2) from an I2C register\n"
    "  wr <bus> <addr> <reg> <b1> [b2]  write 1-2 bytes to an I2C register\n"
    "  ina | ina init            6 current monitors: VBUS, per-leg current, ID register\n"
    "  ntc | ntc init            ADS7128 heatsink NTCs: raw code and temperature\n"
    "  ext                       ADS1015 external temperature probes\n"
    "  a7 r <reg> | a7 w <reg> <val>   ADS7128 register read / write\n"
    "  sense                     one reading of everything (INA + NTC + probes)\n"
    "  watch                     'sense' every second until 'stop'\n"
    "  arm / disarm              allow / forbid DAC output writes\n"
    "  dac zero | dac set <leg|all> <amps> [s] | dac code <leg|all> <hex> [s] | dac ov|oc <hex> | dac ovv <volts> | dac oca <amps>\n"
    "  fan <0-100>               fan PWM duty\n"
    "  fault | faultset <0|1> | faultrst   read HARD_DISABLE, drive MCU_DISABLE, pulse MCU_RESET\n"
    "  sd [init] | sd test | sd log | sd ls   SD card mount, raw read/write, real logger, listing\n"
    "Safety: DAC writes need 'arm', are capped at 1.0 A per leg, and return to 0 after 10 s, on 'stop', or on reboot.");
}

bool devicesCommand(int argc, String argv[]) {
  const String& cmd = argv[0];
  if (cmd == "scan")          cmdScan();
  else if (cmd == "rd")       cmdRd(argc, argv);
  else if (cmd == "wr")       cmdWr(argc, argv);
  else if (cmd == "ina")      cmdIna(argc, argv);
  else if (cmd == "ntc")      cmdNtc(argc, argv);
  else if (cmd == "ext")      cmdExt();
  else if (cmd == "a7")       cmdA7(argc, argv);
  else if (cmd == "sense")    devicesSense();
  else if (cmd == "arm")      { armed = true; Serial.println("ARMED: dac writes enabled. Check the load input is safe. 'disarm' or 'stop' to cancel."); }
  else if (cmd == "disarm")   { devicesSafeState(); Serial.println("disarmed, DAC setpoints = 0"); }
  else if (cmd == "dac")      cmdDac(argc, argv);
  else if (cmd == "fan")      cmdFan(argc, argv);
  else if (cmd == "fault" || cmd == "faultset" || cmd == "faultrst") cmdFault(cmd, argc, argv);
  else if (cmd == "sd")       cmdSd(argc, argv);
  else return false;
  return true;
}

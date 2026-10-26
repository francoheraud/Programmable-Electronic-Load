// Bring-up firmware: ESP32-S3, OLED, rotary encoder + button, back button, real UI. Nothing else is started.
// Build/flash: pio run -e bringup -t upload     Monitor: pio device monitor -e bringup
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Pins.h>
#include <SystemState.h>
#include <Display.h>
#include <UI.h>
#include <UiLogic.h>
#include "devices.h"

// ---- globals the UI library expects (normally defined in src/main.cpp) ----
float overVoltage = 30.0f;
float overCurrent = 30.3f;
float overPower = 500.0f;
float overTemperature = 60.0f;
float setCurrent = 0.0f;
float setVoltage = 0.0f;
float setTime = 0.0f;
uint32_t testTimeElapsed = 0;
uint32_t testStartMs = 0;
float measV_Bus = 0.0f;
float measCurrent[6] = {0, 0, 0, 0, 0, 0};
float totalCurrent = 0.0f;
float measTemp_Int[6] = {0, 0, 0, 0, 0, 0};
float measTemp_Ext[4] = {0, 0, 0, 0};
float measPower = 0.0f;
LoadState loadState = LoadState::IDLE;
FaultReason faultReason = FaultReason::NONE;
bool faultActive = false;

static const uint32_t DEBOUNCE_MS = 40;         // same as lib/UI/UI.cpp
static const int32_t  COUNTS_PER_DETENT = 4;    // same as lib/UI/UI.cpp
static const uint32_t UI_PERIOD_MS = 50;
static const uint32_t SAMPLE_PERIOD_MS = 200;

enum class Mode : uint8_t { NONE, ENC, BTN, UI, WATCH };
static Mode mode = Mode::NONE;
static uint32_t lastWatchMs = 0;
static bool oledOk = false;

// ---- encoder mode (own ISR, same decoder as the UI) ----
static volatile int32_t encTotal = 0;
static volatile uint32_t encInvalid = 0;
static volatile uint8_t encPrev = 0;
static int32_t encLastPrinted = 0;

static void IRAM_ATTR encIsr() {
  uint8_t s = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  int8_t step = decodeQuadrature(encPrev, s);
  if (step == 0 && s != encPrev) { encInvalid++; }
  encTotal += step;
  encPrev = s;
}

// ---- button mode ----
struct WatchedButton {
  const char* name;
  uint8_t pin;
  bool activeHigh;
  ButtonState state;
  bool lastRaw;
  uint32_t rawEdges;
  uint32_t pressMs;
  bool wasStable;
  WatchedButton(const char* n, uint8_t p, bool high)
    : name(n), pin(p), activeHigh(high), lastRaw(false), rawEdges(0), pressMs(0), wasStable(false) {}
};
static WatchedButton confirmBtn("CONFIRM(encoder)", PIN_ENC_BUTTON, true);
static WatchedButton backBtn("BACK", PIN_BACK_BUTTON, false);

static void watchButton(WatchedButton& b, uint32_t now) {
  bool raw = (digitalRead(b.pin) == HIGH) == b.activeHigh;
  if (raw != b.lastRaw) { b.lastRaw = raw; b.rawEdges++; }
  debounceUpdate(b.state, raw, now, DEBOUNCE_MS);
  if (b.state.pressedEdge) {
    b.pressMs = now;
    Serial.printf("%s press   (raw edges while settling: %lu)\n", b.name, (unsigned long)b.rawEdges);
  }
  if (b.wasStable && !b.state.stableLevelPressed) {
    Serial.printf("%s release (held %lu ms)\n", b.name, (unsigned long)(now - b.pressMs));
    b.rawEdges = 0;
  }
  b.wasStable = b.state.stableLevelPressed;
}

static void resetWatch(WatchedButton& b) {
  b.state = ButtonState();
  b.lastRaw = false;
  b.rawEdges = 0;
  b.wasStable = false;
}

// ---- UI mode ----
static int lastScreen = -1;
static int lastLoadState = -1;
static uint32_t lastUiMs = 0;
static uint32_t lastSampleMs = 0;

static const char* screenName(UiScreen s) {
  switch (s) {
    case UiScreen::HOME: return "HOME";
    case UiScreen::SPEC_MENU: return "SPEC_MENU";
    case UiScreen::EDIT_SETPOINT_V: return "EDIT_SETPOINT_V";
    case UiScreen::EDIT_SETPOINT_I: return "EDIT_SETPOINT_I";
    case UiScreen::EDIT_TIMER: return "EDIT_TIMER";
    case UiScreen::EDIT_MAXTEMP: return "EDIT_MAXTEMP";
    case UiScreen::EDIT_OV: return "EDIT_OV";
    case UiScreen::EDIT_OC: return "EDIT_OC";
    case UiScreen::EDIT_OP: return "EDIT_OP";
    case UiScreen::RUN: return "RUN";
    case UiScreen::RUN_ABORT_CONFIRM: return "RUN_ABORT_CONFIRM";
    case UiScreen::SUMMARY: return "SUMMARY";
    case UiScreen::RECENT_LIST: return "RECENT_LIST";
    case UiScreen::RECENT_DETAIL: return "RECENT_DETAIL";
  }
  return "?";
}

static const char* stateName(LoadState s) {
  switch (s) {
    case LoadState::INIT: return "INIT";
    case LoadState::IDLE: return "IDLE";
    case LoadState::RUNNING_CC: return "RUNNING_CC";
    case LoadState::RUNNING_CV: return "RUNNING_CV";
    case LoadState::FAULT: return "FAULT";
  }
  return "?";
}

// Stands in for main.cpp's control loop: run timer plus made-up readings so the Run/Summary pages have data.
static void simulateLoad(uint32_t now) {
  bool running = (loadState == LoadState::RUNNING_CC || loadState == LoadState::RUNNING_CV);
  for (uint8_t i = 0; i < 6; i++) { measTemp_Int[i] = 28.0f + 0.2f * i; }
  for (uint8_t i = 0; i < 4; i++) { measTemp_Ext[i] = 25.0f + i; }

  if (running) {
    testTimeElapsed = now - testStartMs;
    if (testTimeElapsed > (uint32_t)(setTime * 1000.0f)) { loadState = LoadState::IDLE; running = false; }
  }
  if (running) {
    measV_Bus = 12.0f + 0.5f * sinf(now / 1000.0f);
    totalCurrent = setCurrent;
    for (uint8_t i = 0; i < 6; i++) { measCurrent[i] = totalCurrent / 6.0f; }
  } else {
    measV_Bus = 0.0f;
    totalCurrent = 0.0f;
    for (uint8_t i = 0; i < 6; i++) { measCurrent[i] = 0.0f; }
  }
  measPower = measV_Bus * totalCurrent;
}

static void detachEncoderInterrupts() {
  detachInterrupt(digitalPinToInterrupt(PIN_ENC_A));
  detachInterrupt(digitalPinToInterrupt(PIN_ENC_B));
}

static void stopMode() {
  if (mode == Mode::ENC || mode == Mode::UI) { detachEncoderInterrupts(); }
  if (mode != Mode::NONE) { Serial.println("-- stopped --"); }
  mode = Mode::NONE;
}

static void printHelp() {
  Serial.println(
    "Commands:\n"
    "  pins      one-shot raw levels of encoder A/B/button, back button, hard-disable\n"
    "  oled     draw an OLED test pattern (call again to toggle pattern)\n"
    "  enc       stream encoder steps (4 steps = 1 detent); turn the knob\n"
    "  btn       stream debounced button presses/releases for both buttons\n"
    "  ui        run the real UI on the OLED with simulated readings\n"
    "  clearnvs  erase stored test results written by 'ui' runs\n"
    "  stop      leave the current mode (any new command also does)\n"
    "  reboot    restart\n"
    "Tip: in 'ui', set Timer > 0 before Run Test, otherwise the run ends immediately.");
  devicesPrintHelp();
}

static void doPins() {
  Serial.printf("ENC_A(%d)=%d  ENC_B(%d)=%d  ENC_BTN(%d)=%d [pressed=HIGH]  BACK(%d)=%d [pressed=LOW]  HARD_DISABLE(%d)=%d\n",
                PIN_ENC_A, digitalRead(PIN_ENC_A), PIN_ENC_B, digitalRead(PIN_ENC_B),
                PIN_ENC_BUTTON, digitalRead(PIN_ENC_BUTTON), PIN_BACK_BUTTON, digitalRead(PIN_BACK_BUTTON),
                PIN_HARD_DISABLE, digitalRead(PIN_HARD_DISABLE));
}

static void doOled() {
  static bool alt = false;
  if (!oledOk) { Serial.println("OLED did not initialise; run 'scan' and check wiring/address"); return; }
  if (!alt) {
    display.beginFrame();
    display.drawLine(0, "OLED TEST row 0");
    display.drawLine(1, "row 1 normal");
    display.drawLine(2, "row 2 normal");
    display.drawLine(3, "row 3 HIGHLIGHTED", true);
    display.drawLine(4, "row 4 normal");
    display.drawLine(5, "row 5 normal");
    display.drawLine(6, "row 6 normal");
    display.drawLine(7, "123456789012345678901");  // 21 chars = full width
    display.endFrame();
  } else {
    display.showMessage("BRINGUP", "message screen");
  }
  alt = !alt;
  Serial.println("OLED pattern drawn");
}

static void doClearNvs() {
  Preferences prefs;
  prefs.begin("loadtests", false);
  prefs.clear();
  prefs.end();
  Serial.println("stored test results erased (UI counter resets on reboot)");
}

static void startMode(Mode m) {
  stopMode();
  uint32_t now = millis();
  switch (m) {
    case Mode::ENC:
      encTotal = 0;
      encInvalid = 0;
      encLastPrinted = 0;
      encPrev = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
      attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encIsr, CHANGE);
      attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encIsr, CHANGE);
      Serial.println("enc: turn the knob (positive/negative = direction). 'stop' to exit.");
      break;
    case Mode::BTN:
      resetWatch(confirmBtn);
      resetWatch(backBtn);
      Serial.println("btn: press the buttons. 'stop' to exit.");
      break;
    case Mode::WATCH:
      lastWatchMs = now - 1000;
      Serial.println("watch: reading everything once a second. 'stop' to exit.");
      break;
    case Mode::UI:
      ui.begin(PIN_ENC_A, PIN_ENC_B, PIN_ENC_BUTTON, PIN_BACK_BUTTON);
      lastScreen = -1;
      lastLoadState = -1;
      lastUiMs = now;
      lastSampleMs = now;
      Serial.println("ui: use the encoder/buttons. 'stop' to exit.");
      break;
    default:
      break;
  }
  mode = m;
}

static void handleCommand(String line) {
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) return;

  String argv[6];
  int argc = 0;
  int pos = 0;
  while (argc < 6) {
    while (pos < (int)line.length() && line[pos] == ' ') { pos++; }
    if (pos >= (int)line.length()) break;
    int end = line.indexOf(' ', pos);
    if (end < 0) end = line.length();
    argv[argc++] = line.substring(pos, end);
    pos = end;
  }
  const String& cmd = argv[0];

  if (cmd == "help" || cmd == "?") { stopMode(); printHelp(); }
  else if (cmd == "pins")     { stopMode(); doPins(); }
  else if (cmd == "oled")     { stopMode(); doOled(); }
  else if (cmd == "clearnvs") { stopMode(); doClearNvs(); }
  else if (cmd == "enc")      { startMode(Mode::ENC); }
  else if (cmd == "btn")      { startMode(Mode::BTN); }
  else if (cmd == "ui")       { startMode(Mode::UI); }
  else if (cmd == "watch")    { startMode(Mode::WATCH); }
  else if (cmd == "stop")     { stopMode(); devicesSafeState(); Serial.println("Stopped"); }
  else if (cmd == "reboot")   { devicesSafeState(); Serial.println("rebooting"); delay(100); ESP.restart(); }
  else {
    stopMode();
    if (!devicesCommand(argc, argv)) {
      Serial.printf("unknown command '%s' (try 'help')\n", cmd.c_str());
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nBRINGUP firmware (MCU, UI, OLED and device tests; devices start on first use)");

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_BACK_BUTTON, INPUT_PULLUP);
  pinMode(PIN_ENC_BUTTON, INPUT_PULLDOWN);
  pinMode(PIN_HARD_DISABLE, INPUT);

  Wire2.begin(PIN_SDA_2, PIN_SCL_2);
  Wire2.setTimeOut(50);
  oledOk = display.begin();
  Serial.printf("OLED: %s\n", oledOk ? "initialised" : "NOT detected (run 'scan')");
  if (oledOk) { display.showMessage("BRINGUP", "type 'help' on serial"); }

  printHelp();
}

void loop() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') { handleCommand(line); line = ""; }
    else if (line.length() < 64) { line += c; }
  }

  uint32_t now = millis();
  devicesTick(now);
  switch (mode) {
    case Mode::WATCH:
      if (now - lastWatchMs >= 1000) {
        lastWatchMs = now;
        devicesSense();
      }
      break;
    case Mode::ENC: {
      int32_t total = encTotal;
      if (total != encLastPrinted) {
        encLastPrinted = total;
        int32_t detents = total / COUNTS_PER_DETENT;
        Serial.printf("steps=%ld  detents=%ld rem=%ld  invalid-transitions=%lu  A=%d B=%d\n",
                      (long)total, (long)detents, (long)(total - detents * COUNTS_PER_DETENT),
                      (unsigned long)encInvalid, digitalRead(PIN_ENC_A), digitalRead(PIN_ENC_B));
      }
      break;
    }
    case Mode::BTN:
      watchButton(confirmBtn, now);
      watchButton(backBtn, now);
      delay(1);
      break;
    case Mode::UI:
      if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
        lastSampleMs = now;
        simulateLoad(now);
        ui.recordStatsSample();
      }
      if (now - lastUiMs >= UI_PERIOD_MS) {
        lastUiMs = now;
        ui.tick();
        int s = (int)ui.screen();
        if (s != lastScreen) { lastScreen = s; Serial.printf("screen -> %s\n", screenName(ui.screen())); }
        int ls = (int)loadState;
        if (ls != lastLoadState) { lastLoadState = ls; Serial.printf("loadState -> %s\n", stateName(loadState)); }
      }
      break;
    default:
      break;
  }
}

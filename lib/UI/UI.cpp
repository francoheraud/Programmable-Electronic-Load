#include "UI.h"
#include <Display.h>
#include <Preferences.h>
#include <float.h>
#include <LoadSetpoints.h>

UiController ui;

static const uint32_t DEBOUNCE_STABLE_MS = 40;
static const int32_t ENCODER_COUNTS_PER_DETENT = 4;

static const float MAX_UI_CURRENT_A = 30.3f;

static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int32_t encoderAccum = 0;
static volatile uint8_t encoderPrevState = 0;
static uint8_t s_pinEncA = 0;
static uint8_t s_pinEncB = 0;

static void IRAM_ATTR onEncoderChange() {
  uint8_t a = digitalRead(s_pinEncA);
  uint8_t b = digitalRead(s_pinEncB);
  uint8_t newState = (a << 1) | b;
  int8_t step = decodeQuadrature(encoderPrevState, newState);
  portENTER_CRITICAL_ISR(&encMux);
  encoderAccum += step;
  portEXIT_CRITICAL_ISR(&encMux);
  encoderPrevState = newState;
}

int32_t UiController::readEncoderDelta() {
  int32_t raw;
  portENTER_CRITICAL(&encMux);
  raw = encoderAccum;
  encoderAccum = 0;
  portEXIT_CRITICAL(&encMux);
  return raw;
}

int32_t UiController::consumeDetents() {
  return extractDetents(encoderCarry, readEncoderDelta(), ENCODER_COUNTS_PER_DETENT);
}

//button debounce
void UiController::updateButton(ButtonState& btn, bool rawLevel, uint32_t now) {
  debounceUpdate(btn, rawLevel, now, DEBOUNCE_STABLE_MS);
}

void UiController::begin(uint8_t pinEncA, uint8_t pinEncB, uint8_t pinEncButton, uint8_t pinBackButton) {
  s_pinEncA = pinEncA;
  s_pinEncB = pinEncB;
  _pinEncButton = pinEncButton;
  _pinBackButton = pinBackButton;

  encoderPrevState = (digitalRead(s_pinEncA) << 1) | digitalRead(s_pinEncB);
  attachInterrupt(digitalPinToInterrupt(s_pinEncA), onEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(s_pinEncB), onEncoderChange, CHANGE);
}

//test-run stats
void UiController::resetStats() {
  statMinV = FLT_MAX;  statMaxV = 0.0f;  statSumV = 0.0f;
  statMinI = FLT_MAX;  statMaxI = 0.0f;  statSumI = 0.0f;
  statSampleCount = 0;
  statMaxIntTemp = -FLT_MAX;
  statMaxExtTemp = -FLT_MAX;
  runStartMs = millis();
}

void UiController::recordStatsSample() {
  if (!(loadState == LoadState::RUNNING_CC || loadState == LoadState::RUNNING_CV)) return;

  statMinV = min(statMinV, measV_Bus);
  statMaxV = max(statMaxV, measV_Bus);
  statSumV += measV_Bus;
  statMinI = min(statMinI, totalCurrent);
  statMaxI = max(statMaxI, totalCurrent);
  statSumI += totalCurrent;
  statSampleCount++;

  for (uint8_t i = 0; i < 6; i++) { statMaxIntTemp = max(statMaxIntTemp, measTemp_Int[i]); }
  for (uint8_t i = 0; i < 4; i++) {
    if (isfinite(measTemp_Ext[i])) statMaxExtTemp = max(statMaxExtTemp, measTemp_Ext[i]);
  }
}

void UiController::finalizeStats(TestResult result) {
  lastSummary.testIndex = ++testCounter;
  lastSummary.mode = homeMode;
  lastSummary.minVoltage = statSampleCount ? statMinV : 0.0f;
  lastSummary.maxVoltage = statMaxV;
  lastSummary.avgVoltage = statSampleCount ? statSumV / statSampleCount : 0.0f;
  lastSummary.minCurrent = statSampleCount ? statMinI : 0.0f;
  lastSummary.maxCurrent = statMaxI;
  lastSummary.avgCurrent = statSampleCount ? statSumI / statSampleCount : 0.0f;
  lastSummary.maxIntTemp = (statMaxIntTemp == -FLT_MAX) ? 0.0f : statMaxIntTemp;
  lastSummary.maxExtTemp = (statMaxExtTemp == -FLT_MAX) ? NAN : statMaxExtTemp;
  lastSummary.durationMs = millis() - runStartMs;
  lastSummary.result = result;
  lastSummary.fault = (result == TestResult::FAULT) ? faultReason : FaultReason::NONE;

  persistSummary(lastSummary);
  currentScreen = UiScreen::SUMMARY;
  summaryScroll = 0;
  abortRequested = false;
}

//persistence
void UiController::persistSummary(const TestSummary& s) {
  Preferences prefs;
  prefs.begin("loadtests", false);
  uint8_t nextSlot = prefs.getUChar("nextSlot", 0);
  uint8_t count = prefs.getUChar("count", 0);
  char key[8];
  snprintf(key, sizeof(key), "slot%u", nextSlot);
  prefs.putBytes(key, &s, sizeof(TestSummary));
  prefs.putUChar("nextSlot", (nextSlot + 1) % 5);
  if (count < 5) { prefs.putUChar("count", count + 1); }
  prefs.end();
}

uint8_t UiController::loadRecentList(TestSummary out[5]) {
  Preferences prefs;
  prefs.begin("loadtests", true);
  uint8_t nextSlot = prefs.getUChar("nextSlot", 0);
  uint8_t count = prefs.getUChar("count", 0);
  for (uint8_t i = 0; i < count; i++) {
    int8_t slotIdx = ((int8_t)nextSlot - 1 - (int8_t)i + 5) % 5;
    char key[8];
    snprintf(key, sizeof(key), "slot%u", slotIdx);
    prefs.getBytes(key, &out[i], sizeof(TestSummary));
  }
  prefs.end();
  return count;
}

//global run-edge rule
void UiController::handleGlobalRunEdge() {
  bool wasRunning = (prevLoadState == LoadState::RUNNING_CC || prevLoadState == LoadState::RUNNING_CV);
  bool isRunning = (loadState == LoadState::RUNNING_CC || loadState == LoadState::RUNNING_CV);

  if (isRunning && !wasRunning) {
    resetStats();
    currentScreen = UiScreen::RUN;
    runPage = 0;
  } else if (wasRunning && !isRunning) {
    // run ended: fault mid-run reports FAULT regardless of a pending abort request
    TestResult result = (loadState == LoadState::FAULT) ? TestResult::FAULT
                       : (abortRequested ? TestResult::ABORTED : TestResult::COMPLETED);
    finalizeStats(result);
  } else if (!wasRunning && !isRunning &&
             loadState == LoadState::FAULT && prevLoadState != LoadState::FAULT) {
    // fault while idle (no run in progress) -- stats are stale, but still surface the fault
    finalizeStats(TestResult::FAULT);
  }

  prevLoadState = loadState;
}

uint8_t UiController::specMenuRowCount() const {
  return (homeMode == ModeSel::CV) ? 10 : 9;
}

void UiController::enterEdit(UiScreen screen, float currentValue) {
  editBuffer = currentValue;
  if (screen == UiScreen::EDIT_TIMER)
    editBuffer = isfinite(currentValue) ? constrain(currentValue, 0.0f, 86400.0f) : 0;
  if (screen == UiScreen::EDIT_SETPOINT_I)
    editBuffer = LoadSetpoints::advance(currentValue, 0, LoadSetpoints::CURRENT_STEP_A, LoadSetpoints::MAX_CURRENT_A);
  if (screen == UiScreen::EDIT_SETPOINT_V)
    editBuffer = LoadSetpoints::advance(currentValue, 0, LoadSetpoints::VOLTAGE_STEP_V, LoadSetpoints::MAX_VOLTAGE_V);
  if (screen == UiScreen::EDIT_OV)
    editBuffer = constrain(currentValue, 0.0f, LoadSetpoints::MAX_VOLTAGE_V);
  if (screen == UiScreen::EDIT_OP)
    editBuffer = constrain(currentValue, 0.0f, LoadSetpoints::MAX_POWER_W);
  currentScreen = screen;
}

void UiController::onRotate(int32_t detents) {
  if (detents == 0) return;

  switch (currentScreen) {
    case UiScreen::HOME: {
      int8_t next = (int8_t)homeRow + (detents > 0 ? 1 : -1);
      if (next < 0) next = 0;
      if (next > 2) next = 2;
      homeRow = (uint8_t)next;
      break;
    }
    case UiScreen::SPEC_MENU: {
      int8_t count = (int8_t)specMenuRowCount();
      int8_t next = (int8_t)specMenuIndex + (detents > 0 ? 1 : -1);
      if (next < 0) next = 0;
      if (next >= count) next = count - 1;
      specMenuIndex = (uint8_t)next;
      if (specMenuIndex < specMenuScroll) { specMenuScroll = specMenuIndex; }
      if (specMenuIndex > specMenuScroll + 6) { specMenuScroll = specMenuIndex - 6; }
      break;
    }
    case UiScreen::EDIT_SETPOINT_V:
      editBuffer = LoadSetpoints::advance(editBuffer, detents, LoadSetpoints::VOLTAGE_STEP_V, LoadSetpoints::MAX_VOLTAGE_V);
      break;
    case UiScreen::EDIT_SETPOINT_I:
      editBuffer = LoadSetpoints::advanceCurrent(editBuffer, detents, currentStepIndex);
      break;
    case UiScreen::EDIT_TIMER:
      editBuffer += (timerHours ? 3600.0f : 5.0f) * detents;
      editBuffer = constrain(editBuffer, 0.0f, 86400.0f);
      break;
    case UiScreen::EDIT_MAXTEMP:
      editBuffer += 1.0f * detents;
      editBuffer = constrain(editBuffer, 0.0f, 150.0f);
      break;
    case UiScreen::EDIT_OV:
      editBuffer += 0.5f * detents;
      editBuffer = constrain(editBuffer, 0.0f, LoadSetpoints::MAX_VOLTAGE_V);
      break;
    case UiScreen::EDIT_OC:
      editBuffer += 0.1f * detents;
      editBuffer = constrain(editBuffer, 0.0f, MAX_UI_CURRENT_A);
      break;
    case UiScreen::EDIT_OP:
      editBuffer += 5.0f * detents;
      editBuffer = constrain(editBuffer, 0.0f, LoadSetpoints::MAX_POWER_W);
      break;
    case UiScreen::RUN: {
      int8_t next = (int8_t)runPage + (detents > 0 ? 1 : -1);
      if (next < 0) next = 2;
      if (next > 2) next = 0;
      runPage = (uint8_t)next;
      break;
    }
    case UiScreen::RUN_ABORT_CONFIRM:
      break; // no toggle
    case UiScreen::SUMMARY:
    case UiScreen::RECENT_DETAIL: {
      int8_t next = (int8_t)summaryScroll + (detents > 0 ? 1 : -1);
      if (next < 0) next = 0;
      if (next > 1) next = 1;
      summaryScroll = (uint8_t)next;
      break;
    }
    case UiScreen::RECENT_LIST: {
      if (recentCount == 0) break;
      int8_t next = (int8_t)recentListIndex + (detents > 0 ? 1 : -1);
      if (next < 0) next = 0;
      if (next >= (int8_t)recentCount) next = recentCount - 1;
      recentListIndex = (uint8_t)next;
      break;
    }
  }
}

void UiController::onConfirm() {
  switch (currentScreen) {
    case UiScreen::HOME:
      if (homeRow == 0) {
        homeMode = (homeMode == ModeSel::CC) ? ModeSel::CV : ModeSel::CC;
      } else if (homeRow == 1) {
        currentScreen = UiScreen::SPEC_MENU;
        specMenuIndex = 0;
        specMenuScroll = 0;
      } else if (homeRow == 2) {
        recentCount = loadRecentList(recentCache);
        currentScreen = UiScreen::RECENT_LIST;
        recentListIndex = 0;
      }
      break;

    case UiScreen::SPEC_MENU: {
      uint8_t rowCount = specMenuRowCount();
      bool isCV = (homeMode == ModeSel::CV);
      uint8_t runTestRow = rowCount - 1;

      if (specMenuIndex == runTestRow) {
        if (faultActive || loadState == LoadState::FAULT) { break; }
        testStartMs = millis();  // set before the state change so the control loop never sees a stale time
        testTimeElapsed = 0;
        loadState = isCV ? LoadState::RUNNING_CV : LoadState::RUNNING_CC;
      } else if (specMenuIndex == runTestRow - 1) {
        currentStepIndex = (currentStepIndex + 1) % 3;
      } else if (specMenuIndex == runTestRow - 2) {
        timerHours = !timerHours;
      } else if (isCV && specMenuIndex == 0) {
        enterEdit(UiScreen::EDIT_SETPOINT_V, setVoltage);
      } else if (isCV && specMenuIndex == 1) {
        enterEdit(UiScreen::EDIT_SETPOINT_I, setCurrent);
      } else if (!isCV && specMenuIndex == 0) {
        enterEdit(UiScreen::EDIT_SETPOINT_I, setCurrent);
      } else {
        uint8_t row = specMenuIndex - (isCV ? 2 : 1);
        switch (row) {
          case 0: enterEdit(UiScreen::EDIT_TIMER, setTime); break;
          case 1: enterEdit(UiScreen::EDIT_MAXTEMP, overTemperature); break;
          case 2: enterEdit(UiScreen::EDIT_OV, overVoltage); break;
          case 3: enterEdit(UiScreen::EDIT_OC, overCurrent); break;
          case 4: enterEdit(UiScreen::EDIT_OP, overPower); break;
        }
      }
      break;
    }

    case UiScreen::EDIT_SETPOINT_V:
      setVoltage = editBuffer;
      currentScreen = UiScreen::SPEC_MENU;
      break;
    case UiScreen::EDIT_SETPOINT_I:
      setCurrent = editBuffer;
      currentScreen = UiScreen::SPEC_MENU;
      break;
    case UiScreen::EDIT_TIMER:
      setTime = editBuffer;
      currentScreen = UiScreen::SPEC_MENU;
      break;
    case UiScreen::EDIT_MAXTEMP:
      overTemperature = editBuffer;
      currentScreen = UiScreen::SPEC_MENU;
      break;
    case UiScreen::EDIT_OV:
      overVoltage = editBuffer;
      currentScreen = UiScreen::SPEC_MENU;
      break;
    case UiScreen::EDIT_OC:
      overCurrent = editBuffer;
      currentScreen = UiScreen::SPEC_MENU;
      break;
    case UiScreen::EDIT_OP:
      overPower = editBuffer;
      currentScreen = UiScreen::SPEC_MENU;
      break;

    case UiScreen::RUN:
      break;

    case UiScreen::RUN_ABORT_CONFIRM:
      abortRequested = true;
      loadState = LoadState::IDLE;
      break;

    case UiScreen::SUMMARY:
      break;

    case UiScreen::RECENT_LIST:
      if (recentCount > 0) {
        currentScreen = UiScreen::RECENT_DETAIL;
        summaryScroll = 0;
      }
      break;

    case UiScreen::RECENT_DETAIL:
      break;
  }
}

void UiController::onBack() {
  switch (currentScreen) {
    case UiScreen::HOME:
      break;

    case UiScreen::SPEC_MENU:
      currentScreen = UiScreen::HOME;
      break;

    case UiScreen::EDIT_SETPOINT_V:
    case UiScreen::EDIT_SETPOINT_I:
    case UiScreen::EDIT_TIMER:
    case UiScreen::EDIT_MAXTEMP:
    case UiScreen::EDIT_OV:
    case UiScreen::EDIT_OC:
    case UiScreen::EDIT_OP:
      currentScreen = UiScreen::SPEC_MENU;
      break;

    case UiScreen::RUN:
      currentScreen = UiScreen::RUN_ABORT_CONFIRM;
      break;

    case UiScreen::RUN_ABORT_CONFIRM:
      currentScreen = UiScreen::RUN;
      break;

    case UiScreen::SUMMARY:
      currentScreen = UiScreen::HOME;
      break;

    case UiScreen::RECENT_LIST:
      currentScreen = UiScreen::HOME;
      break;

    case UiScreen::RECENT_DETAIL:
      currentScreen = UiScreen::RECENT_LIST;
      break;
  }
}

void UiController::tick() {
  uint32_t now = millis();

  handleGlobalRunEdge();

  int32_t detents = consumeDetents();
  updateButton(confirmBtn, digitalRead(_pinEncButton) == HIGH, now);
  updateButton(backBtn, digitalRead(_pinBackButton) == LOW, now);

  if (detents != 0) { onRotate(detents); }
  if (confirmBtn.pressedEdge) { onConfirm(); }
  if (backBtn.pressedEdge) { onBack(); }

  render();
}

// --- rendering ---
static const char* faultReasonText(FaultReason f) {
  switch (f) {
    case FaultReason::NONE: return "None";
    case FaultReason::INT_TEMP: return "Int Temp";
    case FaultReason::EXT_TEMP: return "Ext Temp";
    case FaultReason::VOLTAGE: return "Voltage";
    case FaultReason::CURRENT: return "Current";
    case FaultReason::POWER: return "Power";
    case FaultReason::SENSOR_UNRESPONSIVE: return "Sensor";
    case FaultReason::SELF_TEST: return "Self-test";
    default: return "?";
  }
}

void UiController::render() {
  display.beginFrame();
  switch (currentScreen) {
    case UiScreen::HOME: renderHome(); break;
    case UiScreen::SPEC_MENU: renderSpecMenu(); break;
    case UiScreen::EDIT_SETPOINT_V:
    case UiScreen::EDIT_SETPOINT_I:
    case UiScreen::EDIT_TIMER:
    case UiScreen::EDIT_MAXTEMP:
    case UiScreen::EDIT_OV:
    case UiScreen::EDIT_OC:
    case UiScreen::EDIT_OP:
      renderEdit();
      break;
    case UiScreen::RUN: renderRun(); break;
    case UiScreen::RUN_ABORT_CONFIRM: renderAbortConfirm(); break;
    case UiScreen::SUMMARY: renderSummary(lastSummary, summaryScroll); break;
    case UiScreen::RECENT_LIST: renderRecentList(); break;
    case UiScreen::RECENT_DETAIL: renderRecentDetail(); break;
  }
  display.endFrame();
}

void UiController::renderHome() {
  char line[32];
  bool faulted = faultActive;

  display.drawLine(0, faulted ? "STATUS: FAULT" : "STATUS: OK");
  if (isfinite(measTemp_Ext[0]))
    snprintf(line, sizeof(line), "Int %.0fC Ext %.0fC", measTemp_Int[0], measTemp_Ext[0]);
  else
    snprintf(line, sizeof(line), "Int %.0fC Ext --", measTemp_Int[0]);
  display.drawLine(1, line);

  snprintf(line, sizeof(line), "Mode: %s", homeMode == ModeSel::CC ? "CC" : "CV");
  display.drawLine(3, line, homeRow == 0);
  display.drawLine(4, "Test Parameters", homeRow == 1);
  display.drawLine(5, "Recent Tests", homeRow == 2);
}

void UiController::renderSpecMenu() {
  bool isCV = (homeMode == ModeSel::CV);
  char rows[10][24];
  uint8_t count = 0;

  if (isCV) {
    snprintf(rows[count++], 24, "Setpoint (V): %.2f", setVoltage);
    snprintf(rows[count++], 24, "Current Limit: %.2f", setCurrent);
  } else {
    snprintf(rows[count++], 24, "Setpoint (I): %.2f", setCurrent);
  }
  const uint32_t duration = (uint32_t)setTime;
  snprintf(rows[count++], 24, "Timer: %02lu:%02lu:%02lu", (unsigned long)(duration / 3600),
           (unsigned long)((duration / 60) % 60), (unsigned long)(duration % 60));
  snprintf(rows[count++], 24, "Max Temp: %.0fC", overTemperature);
  snprintf(rows[count++], 24, "Over Voltage: %.1f", overVoltage);
  snprintf(rows[count++], 24, "Over Current: %.1f", overCurrent);
  snprintf(rows[count++], 24, "Over Power: %.0f", overPower);
  snprintf(rows[count++], 24, "Time unit: %s", timerHours ? "Hours" : "Seconds");
  snprintf(rows[count++], 24, "I step: %s", currentStepIndex == 0 ? "0.01A Fine" : (currentStepIndex == 1 ? "0.10A" : "1.00A Coarse"));
  snprintf(rows[count++], 24, "Run Test");

  display.drawLine(0, "TEST PARAMETERS");
  for (uint8_t i = 0; i < 7 && (specMenuScroll + i) < count; i++) {
    uint8_t idx = specMenuScroll + i;
    display.drawLine(i + 1, rows[idx], idx == specMenuIndex);
  }
}

void UiController::renderEdit() {
  const char* label = "Value";
  const char* units = "";
  switch (currentScreen) {
    case UiScreen::EDIT_SETPOINT_V: label = "Setpoint Voltage"; units = "V"; break;
    case UiScreen::EDIT_SETPOINT_I: label = "Setpoint Current"; units = "A"; break;
    case UiScreen::EDIT_TIMER: label = "Timer"; units = "s"; break;
    case UiScreen::EDIT_MAXTEMP: label = "Max Ext Temp"; units = "C"; break;
    case UiScreen::EDIT_OV: label = "Over Voltage"; units = "V"; break;
    case UiScreen::EDIT_OC: label = "Over Current"; units = "A"; break;
    case UiScreen::EDIT_OP: label = "Over Power"; units = "W"; break;
    default: break;
  }

  char line[24];
  display.drawLine(0, label);
  snprintf(line, sizeof(line), "%.2f %s", editBuffer, units);
  if (currentScreen == UiScreen::EDIT_TIMER) {
    const uint32_t duration = (uint32_t)editBuffer;
    snprintf(line, sizeof(line), "%02lu:%02lu:%02lu", (unsigned long)(duration / 3600),
             (unsigned long)((duration / 60) % 60), (unsigned long)(duration % 60));
    display.drawLine(3, "Hours:Minutes:Seconds");
    display.drawLine(4, timerHours ? "Step: 1 hour" : "Step: 5 seconds");
    display.drawLine(5, "Maximum: 24 hours");
  }
  display.drawLine(2, line);
  if (currentScreen == UiScreen::EDIT_SETPOINT_I) {
    display.drawLine(3, currentStepIndex == 0 ? "Step: 0.01 A (10 mA)" : (currentStepIndex == 1 ? "Step: 0.10 A" : "Step: 1.00 A"));
    display.drawLine(4, "Range: 0-30 A");
  } else if (currentScreen == UiScreen::EDIT_SETPOINT_V) {
    display.drawLine(3, "Step: 0.1 V");
    display.drawLine(4, "Range: 0-30 V");
  }
  display.drawLine(6, "Confirm: save");
  display.drawLine(7, "Back: cancel");
}

void UiController::renderRun() {
  char line[24];
  const char* modeStr = (loadState == LoadState::RUNNING_CV) ? "CV" : "CC";

  if (runPage == 0) {
    snprintf(line, sizeof(line), "RUN (%s) pg1/3", modeStr);
    display.drawLine(0, line);
    snprintf(line, sizeof(line), "V: %.2f  I: %.2f", measV_Bus, totalCurrent);
    display.drawLine(1, line);
    snprintf(line, sizeof(line), "P: %.1fW", measPower);
    display.drawLine(2, line);
    uint32_t remainMs = (uint32_t)(setTime * 1000.0f) > testTimeElapsed
                         ? (uint32_t)(setTime * 1000.0f) - testTimeElapsed : 0;
    const uint32_t remaining = remainMs / 1000;
    snprintf(line, sizeof(line), "Remain: %02lu:%02lu:%02lu", (unsigned long)(remaining / 3600),
             (unsigned long)((remaining / 60) % 60), (unsigned long)(remaining % 60));
    display.drawLine(3, line);
  } else if (runPage == 1) {
    display.drawLine(0, "RUN - Temps pg2/3");
    for (uint8_t pair = 0; pair < 3; ++pair) {
      const uint8_t i = pair * 2;
      snprintf(line, sizeof(line), "I%u:%.1f I%u:%.1f", i + 1, measTemp_Int[i], i + 2, measTemp_Int[i + 1]);
      display.drawLine(pair + 1, line);
    }
    for (uint8_t pair = 0; pair < 2; ++pair) {
      const uint8_t i = pair * 2;
      char a[8], b[8];
      if (isfinite(measTemp_Ext[i])) snprintf(a, sizeof(a), "%.1f", measTemp_Ext[i]);
      else snprintf(a, sizeof(a), "--");
      if (isfinite(measTemp_Ext[i + 1])) snprintf(b, sizeof(b), "%.1f", measTemp_Ext[i + 1]);
      else snprintf(b, sizeof(b), "--");
      snprintf(line, sizeof(line), "E%u:%s E%u:%s", i + 1, a, i + 2, b);
      display.drawLine(pair + 4, line);
    }
    display.drawLine(6, "I=internal E=ext (C)");
  } else {
    display.drawLine(0, "RUN - Legs pg3/3");
    snprintf(line, sizeof(line), "%.2f %.2f %.2f", measCurrent[0], measCurrent[1], measCurrent[2]);
    display.drawLine(1, line);
    snprintf(line, sizeof(line), "%.2f %.2f %.2f", measCurrent[3], measCurrent[4], measCurrent[5]);
    display.drawLine(2, line);
  }
  display.drawLine(7, "Back: abort test");
}

void UiController::renderAbortConfirm() {
  display.drawLine(0, "ABORT TEST?");
  display.drawLine(3, "Confirm = Yes, abort");
  display.drawLine(4, "Back    = No, resume");
}

void UiController::renderSummary(const TestSummary& s, uint8_t scroll) {
  char lines[10][24];
  const char* resultStr = s.result == TestResult::COMPLETED ? "COMPLETED"
                         : s.result == TestResult::ABORTED ? "ABORTED" : "FAULT";

  snprintf(lines[0], 24, "Test #%lu: %s", (unsigned long)s.testIndex, resultStr);
  snprintf(lines[1], 24, "Mode: %s  Dur: %lus", s.mode == ModeSel::CC ? "CC" : "CV",
           (unsigned long)(s.durationMs / 1000));
  snprintf(lines[2], 24, "V min/avg/max");
  snprintf(lines[3], 24, "%.2f/%.2f/%.2f", s.minVoltage, s.avgVoltage, s.maxVoltage);
  snprintf(lines[4], 24, "I min/avg/max");
  snprintf(lines[5], 24, "%.2f/%.2f/%.2f", s.minCurrent, s.avgCurrent, s.maxCurrent);
  snprintf(lines[6], 24, "Max Int/Ext Temp");
  if (isfinite(s.maxExtTemp)) snprintf(lines[7], 24, "%.0fC / %.0fC", s.maxIntTemp, s.maxExtTemp);
  else snprintf(lines[7], 24, "%.0fC / --", s.maxIntTemp);
  snprintf(lines[8], 24, "Fault: %s", faultReasonText(s.fault));

  display.drawLine(0, lines[0]);
  for (uint8_t i = 0; i < 7; i++) {
    uint8_t idx = 1 + scroll + i;
    if (idx < 9) { display.drawLine(i + 1, lines[idx]); }
  }
}

void UiController::renderRecentList() {
  display.drawLine(0, "RECENT TESTS");
  if (recentCount == 0) {
    display.drawLine(2, "No recent tests");
    return;
  }
  for (uint8_t i = 0; i < recentCount && i < 6; i++) {
    char line[24];
    const char* resultStr = recentCache[i].result == TestResult::COMPLETED ? "OK"
                           : recentCache[i].result == TestResult::ABORTED ? "ABORT" : "FAULT";
    snprintf(line, sizeof(line), "#%lu %s %s", (unsigned long)recentCache[i].testIndex,
             recentCache[i].mode == ModeSel::CC ? "CC" : "CV", resultStr);
    display.drawLine(i + 1, line, i == recentListIndex);
  }
}

void UiController::renderRecentDetail() {
  if (recentListIndex < recentCount) {
    renderSummary(recentCache[recentListIndex], summaryScroll);
  }
}

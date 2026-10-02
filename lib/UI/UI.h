#pragma once
#include <Arduino.h>
#include <SystemState.h>

enum class UiScreen : uint8_t {
  HOME,
  SPEC_MENU,          // on-screen label: "Test Parameters"
  EDIT_SETPOINT_V,
  EDIT_SETPOINT_I,
  EDIT_TIMER,
  EDIT_MAXTEMP,       // edits overTemperature (external trip limit)
  EDIT_OV,
  EDIT_OC,
  EDIT_OP,
  RUN,
  RUN_ABORT_CONFIRM,
  SUMMARY,
  RECENT_LIST,
  RECENT_DETAIL
};

enum class TestResult : uint8_t { COMPLETED, ABORTED, FAULT };
enum class ModeSel : uint8_t { CC, CV };

struct TestSummary {
  uint32_t testIndex = 0;
  ModeSel mode = ModeSel::CC;
  float minVoltage = 0.0f, maxVoltage = 0.0f, avgVoltage = 0.0f;
  float minCurrent = 0.0f, maxCurrent = 0.0f, avgCurrent = 0.0f;
  float maxIntTemp = 0.0f, maxExtTemp = 0.0f;
  uint32_t durationMs = 0;
  TestResult result = TestResult::COMPLETED;
  FaultReason fault = FaultReason::NONE;
};

class UiController {
public:
  void begin(uint8_t pinEncA, uint8_t pinEncB, uint8_t pinEncButton, uint8_t pinBackButton);
  void tick();
  void recordStatsSample();

private:
  struct ButtonState {
    bool rawPressed = false;
    bool stableLevelPressed = false;
    bool pressedEdge = false;
    uint32_t lastChangeMs = 0;
  };

  // --- navigation state ---
  UiScreen currentScreen = UiScreen::HOME;
  uint8_t homeRow = 0;             // 0=Mode,1=Test Parameters,2=Recent Tests
  ModeSel homeMode = ModeSel::CC;
  uint8_t specMenuIndex = 0;
  uint8_t specMenuScroll = 0;
  uint8_t runPage = 0;             // 0=overview,1=temps,2=per-leg currents
  uint8_t summaryScroll = 0;
  uint8_t recentListIndex = 0;
  float editBuffer = 0.0f;

  // --- encoder ---
  int32_t encoderCarry = 0;
  int32_t readEncoderDelta();
  int32_t consumeDetents();

  // --- buttons ---
  ButtonState confirmBtn, backBtn;
  uint8_t _pinEncButton = 0;
  uint8_t _pinBackButton = 0;
  void updateButton(ButtonState& btn, bool rawLevel, uint32_t now);

  // --- test-run stats ---
  float statMinV = 0.0f, statMaxV = 0.0f, statSumV = 0.0f;
  float statMinI = 0.0f, statMaxI = 0.0f, statSumI = 0.0f;
  uint32_t statSampleCount = 0;
  float statMaxIntTemp = 0.0f, statMaxExtTemp = 0.0f;
  uint32_t runStartMs = 0;
  uint32_t testCounter = 0;
  LoadState prevLoadState = LoadState::INIT;
  bool abortRequested = false;
  TestSummary lastSummary{};
  TestSummary recentCache[5]{};
  uint8_t recentCount = 0;
  void resetStats();
  void finalizeStats(TestResult result);

  // --- persistence (Preferences / NVS) ---
  void persistSummary(const TestSummary& s);
  uint8_t loadRecentList(TestSummary out[5]);

  // --- per-screen input handlers ---
  void handleGlobalRunEdge();
  void onRotate(int32_t detents);
  void onConfirm();
  void onBack();

  uint8_t specMenuRowCount() const;
  void enterEdit(UiScreen screen, float currentValue);

  // --- per-screen render ---
  void render();
  void renderHome();
  void renderSpecMenu();
  void renderEdit();
  void renderRun();
  void renderAbortConfirm();
  void renderSummary(const TestSummary& s, uint8_t scroll);
  void renderRecentList();
  void renderRecentDetail();
};

extern UiController ui;

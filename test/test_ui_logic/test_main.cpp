// Native (host) tests for the UI helpers and UiController state machine.
// Run with: pio test -e native
#include <unity.h>
#include <UI.h>
#include <Display.h>
#include <Preferences.h>
#include <LoadSetpoints.h>

// ---- globals normally defined in main.cpp ----
float overVoltage = 30.0f, overCurrent = 30.3f, overPower = 500.0f, overTemperature = 60.0f;
float setCurrent = 0.0f, setVoltage = 0.0f, setTime = 0.0f;
uint32_t testTimeElapsed = 0, testStartMs = 0;
float measV_Bus = 0.0f, totalCurrent = 0.0f, measPower = 0.0f;
float measCurrent[6] = {0}, measTemp_Int[6] = {0}, measTemp_Ext[4] = {0};
LoadState loadState = LoadState::IDLE;
FaultReason faultReason = FaultReason::NONE;
bool faultActive = false;

static const uint8_t PIN_A = 1, PIN_B = 2, PIN_CONFIRM = 3, PIN_BACK = 4;
static const float MAX_UI_CURRENT_A = 30.3f;  // mirrors lib/UI/UI.cpp

// Friend of UiController under -DUNIT_TEST; the only route into its private state.
struct UiTestAccess {
  static UiScreen& screen()          { return ui.currentScreen; }
  static uint8_t&  homeRow()         { return ui.homeRow; }
  static ModeSel&  homeMode()        { return ui.homeMode; }
  static uint8_t&  specMenuIndex()   { return ui.specMenuIndex; }
  static uint8_t&  specMenuScroll()  { return ui.specMenuScroll; }
  static uint8_t&  runPage()         { return ui.runPage; }
  static uint8_t&  summaryScroll()   { return ui.summaryScroll; }
  static uint8_t&  recentListIndex() { return ui.recentListIndex; }
  static uint8_t&  recentCount()     { return ui.recentCount; }
  static float&    editBuffer()      { return ui.editBuffer; }
  static bool&     abortRequested()  { return ui.abortRequested; }
  static LoadState& prevLoadState()  { return ui.prevLoadState; }
  static TestSummary& lastSummary()  { return ui.lastSummary; }
  static TestSummary* recentCache()  { return ui.recentCache; }
  static uint8_t   specMenuRowCount() { return ui.specMenuRowCount(); }

  static void rotate(int32_t d)  { ui.onRotate(d); }
  static void confirm()          { ui.onConfirm(); }
  static void back()             { ui.onBack(); }
  static void runEdge()          { ui.handleGlobalRunEdge(); }
  static void enterEdit(UiScreen s, float v) { ui.enterEdit(s, v); }
  static void persist(const TestSummary& s)  { ui.persistSummary(s); }
  static uint8_t loadRecent(TestSummary out[5]) { return ui.loadRecentList(out); }
};

#define EXPECT_SCREEN(s) TEST_ASSERT_EQUAL_INT((int)(s), (int)UiTestAccess::screen())
#define EXPECT_STATE(s)  TEST_ASSERT_EQUAL_INT((int)(s), (int)loadState)
#define NEAR(exp, act)   TEST_ASSERT_FLOAT_WITHIN(1e-4f, (exp), (act))

void setUp(void) {
  overVoltage = 30.0f; overCurrent = 30.3f; overPower = 500.0f; overTemperature = 60.0f;
  setCurrent = setVoltage = setTime = 0.0f;
  testTimeElapsed = testStartMs = 0;
  measV_Bus = totalCurrent = measPower = 0.0f;
  loadState = LoadState::IDLE;
  faultReason = FaultReason::NONE;
  faultActive = false;
  g_nvs.clear();
  g_millis = 0;
  memset(g_pinLevel, 0, sizeof(g_pinLevel));
  g_pinLevel[PIN_BACK] = HIGH;  // back button is active-low, released
  ui = UiController();
  ui.begin(PIN_A, PIN_B, PIN_CONFIRM, PIN_BACK);
}
void tearDown(void) {}

// ---------------- decodeQuadrature ----------------

void test_quadrature_directions() {
  const uint8_t fwd[5] = {0, 1, 3, 2, 0};  // A/B Gray sequence
  int sum = 0;
  for (int i = 0; i < 4; i++) sum += decodeQuadrature(fwd[i], fwd[i + 1]);
  TEST_ASSERT_EQUAL_INT(-4, sum);

  sum = 0;
  for (int i = 4; i > 0; i--) sum += decodeQuadrature(fwd[i], fwd[i - 1]);
  TEST_ASSERT_EQUAL_INT(4, sum);
}

void test_quadrature_full_table_properties() {
  for (uint8_t a = 0; a < 4; a++) {
    for (uint8_t b = 0; b < 4; b++) {
      int8_t s = decodeQuadrature(a, b);
      TEST_ASSERT_EQUAL_INT(-s, decodeQuadrature(b, a));  // reverse move negates
      if (a == b) TEST_ASSERT_EQUAL_INT(0, s);            // no movement
      if ((a ^ b) == 3) TEST_ASSERT_EQUAL_INT(0, s);      // both bits changed = skipped edge
      if ((a ^ b) == 1 || (a ^ b) == 2) TEST_ASSERT_TRUE(s == 1 || s == -1);
    }
  }
}

// ---------------- extractDetents ----------------

void test_detents_carry_remainder() {
  int32_t carry = 0;
  TEST_ASSERT_EQUAL_INT(0, extractDetents(carry, 3, 4));
  TEST_ASSERT_EQUAL_INT(3, carry);
  TEST_ASSERT_EQUAL_INT(2, extractDetents(carry, 5, 4));  // 8 counts -> 2 detents
  TEST_ASSERT_EQUAL_INT(0, carry);
}

void test_detents_negative() {
  int32_t carry = 0;
  TEST_ASSERT_EQUAL_INT(-1, extractDetents(carry, -5, 4));
  TEST_ASSERT_EQUAL_INT(-1, carry);
  TEST_ASSERT_EQUAL_INT(-1, extractDetents(carry, -3, 4));
  TEST_ASSERT_EQUAL_INT(0, carry);
}

void test_detents_back_and_forth_cancels() {
  int32_t carry = 0;
  TEST_ASSERT_EQUAL_INT(0, extractDetents(carry, 3, 4));
  TEST_ASSERT_EQUAL_INT(0, extractDetents(carry, -3, 4));
  TEST_ASSERT_EQUAL_INT(0, carry);
}

// ---------------- debounceUpdate ----------------

void test_debounce_single_edge_after_stable() {
  ButtonState b;
  debounceUpdate(b, true, 0, 40);
  TEST_ASSERT_FALSE(b.pressedEdge);
  debounceUpdate(b, true, 39, 40);
  TEST_ASSERT_FALSE(b.pressedEdge);
  debounceUpdate(b, true, 40, 40);
  TEST_ASSERT_TRUE(b.pressedEdge);
  debounceUpdate(b, true, 41, 40);
  TEST_ASSERT_FALSE(b.pressedEdge);  // held: no repeat
  debounceUpdate(b, true, 500, 40);
  TEST_ASSERT_FALSE(b.pressedEdge);
}

void test_debounce_rejects_bounce() {
  ButtonState b;
  uint32_t t = 0;
  for (int i = 0; i < 10; i++) {  // toggles every 10 ms, never stable for 40 ms
    debounceUpdate(b, (i % 2) == 0, t, 40);
    TEST_ASSERT_FALSE(b.pressedEdge);
    t += 10;
  }
  TEST_ASSERT_FALSE(b.stableLevelPressed);
}

void test_debounce_release_gives_no_edge_and_repress_works() {
  ButtonState b;
  debounceUpdate(b, true, 0, 40);
  debounceUpdate(b, true, 40, 40);
  TEST_ASSERT_TRUE(b.stableLevelPressed);
  debounceUpdate(b, false, 100, 40);
  debounceUpdate(b, false, 140, 40);
  TEST_ASSERT_FALSE(b.stableLevelPressed);
  TEST_ASSERT_FALSE(b.pressedEdge);
  debounceUpdate(b, true, 200, 40);
  debounceUpdate(b, true, 240, 40);
  TEST_ASSERT_TRUE(b.pressedEdge);
}

void test_debounce_handles_millis_rollover() {
  ButtonState b;
  uint32_t t0 = 0xFFFFFFF0u;
  debounceUpdate(b, true, t0, 40);
  debounceUpdate(b, true, t0 + 40, 40);  // wraps past zero
  TEST_ASSERT_TRUE(b.pressedEdge);
}

// ---------------- HOME / SPEC_MENU navigation ----------------

// Menu screens move one row per onRotate call regardless of how many detents arrived.
static void spin(int32_t steps) {
  for (int32_t i = 0; i < (steps < 0 ? -steps : steps); i++) UiTestAccess::rotate(steps < 0 ? -1 : 1);
}

void test_home_rotate_clamps() {
  spin(50);
  TEST_ASSERT_EQUAL_INT(2, UiTestAccess::homeRow());
  spin(-50);
  TEST_ASSERT_EQUAL_INT(0, UiTestAccess::homeRow());
}

void test_menus_move_one_row_per_rotate_call() {
  UiTestAccess::rotate(5);
  TEST_ASSERT_EQUAL_INT(1, UiTestAccess::homeRow());
  UiTestAccess::rotate(-5);
  TEST_ASSERT_EQUAL_INT(0, UiTestAccess::homeRow());
}

void test_home_confirm_toggles_mode() {
  UiTestAccess::homeRow() = 0;
  UiTestAccess::confirm();
  TEST_ASSERT_EQUAL_INT((int)ModeSel::CV, (int)UiTestAccess::homeMode());
  UiTestAccess::confirm();
  TEST_ASSERT_EQUAL_INT((int)ModeSel::CC, (int)UiTestAccess::homeMode());
}

void test_home_to_spec_menu_and_back() {
  UiTestAccess::homeRow() = 1;
  UiTestAccess::confirm();
  EXPECT_SCREEN(UiScreen::SPEC_MENU);
  UiTestAccess::back();
  EXPECT_SCREEN(UiScreen::HOME);
  UiTestAccess::back();  // back on HOME does nothing
  EXPECT_SCREEN(UiScreen::HOME);
}

void test_home_to_recent_list_empty() {
  UiTestAccess::homeRow() = 2;
  UiTestAccess::confirm();
  EXPECT_SCREEN(UiScreen::RECENT_LIST);
  TEST_ASSERT_EQUAL_INT(0, UiTestAccess::recentCount());
  UiTestAccess::rotate(3);  // must not move or crash with no entries
  TEST_ASSERT_EQUAL_INT(0, UiTestAccess::recentListIndex());
  UiTestAccess::confirm();  // nothing to open
  EXPECT_SCREEN(UiScreen::RECENT_LIST);
}

void test_spec_menu_row_count() {
  UiTestAccess::homeMode() = ModeSel::CC;
  TEST_ASSERT_EQUAL_INT(9, UiTestAccess::specMenuRowCount());
  UiTestAccess::homeMode() = ModeSel::CV;
  TEST_ASSERT_EQUAL_INT(10, UiTestAccess::specMenuRowCount());
}

void test_spec_menu_rotate_clamps_and_scrolls() {
  UiTestAccess::homeMode() = ModeSel::CV;
  UiTestAccess::screen() = UiScreen::SPEC_MENU;
  spin(100);
  TEST_ASSERT_EQUAL_INT(9, UiTestAccess::specMenuIndex());
  TEST_ASSERT_EQUAL_INT(3, UiTestAccess::specMenuScroll());  // index - 6
  spin(-100);
  TEST_ASSERT_EQUAL_INT(0, UiTestAccess::specMenuIndex());
  TEST_ASSERT_EQUAL_INT(0, UiTestAccess::specMenuScroll());
}

struct EditCase { UiScreen screen; float step; float max; float* target; };

static EditCase editCases[] = {
  {UiScreen::EDIT_SETPOINT_V, 0.1f,  30.0f,        &setVoltage},
  {UiScreen::EDIT_SETPOINT_I, 0.01f, 30.0f, &setCurrent},
  {UiScreen::EDIT_TIMER,      5.0f,  86400.0f,      &setTime},
  {UiScreen::EDIT_MAXTEMP,    1.0f,  150.0f,        &overTemperature},
  {UiScreen::EDIT_OV,         0.5f,  30.0f,        &overVoltage},
  {UiScreen::EDIT_OC,         0.1f,  MAX_UI_CURRENT_A, &overCurrent},
  {UiScreen::EDIT_OP,         5.0f,  500.0f,       &overPower},
};

void test_edit_screens_step_clamp_save() {
  for (const EditCase& c : editCases) {
    UiTestAccess::enterEdit(c.screen, 0.0f);
    UiTestAccess::rotate(1);
    NEAR(c.step, UiTestAccess::editBuffer());
    UiTestAccess::rotate(1000000);
    NEAR(c.max, UiTestAccess::editBuffer());
    UiTestAccess::rotate(-1000000);
    NEAR(0.0f, UiTestAccess::editBuffer());

    UiTestAccess::rotate(2);
    UiTestAccess::confirm();
    NEAR(2 * c.step, *c.target);
    EXPECT_SCREEN(UiScreen::SPEC_MENU);
  }
}

void test_edit_back_cancels_without_saving() {
  setCurrent = 1.0f;
  UiTestAccess::enterEdit(UiScreen::EDIT_SETPOINT_I, setCurrent);
  UiTestAccess::rotate(20);
  UiTestAccess::back();
  NEAR(1.0f, setCurrent);
  EXPECT_SCREEN(UiScreen::SPEC_MENU);
}

void test_cc_menu_row_mapping() {
  UiTestAccess::homeMode() = ModeSel::CC;
  const UiScreen expected[6] = {UiScreen::EDIT_SETPOINT_I, UiScreen::EDIT_TIMER, UiScreen::EDIT_MAXTEMP,
                                UiScreen::EDIT_OV, UiScreen::EDIT_OC, UiScreen::EDIT_OP};
  for (uint8_t i = 0; i < 6; i++) {
    UiTestAccess::screen() = UiScreen::SPEC_MENU;
    UiTestAccess::specMenuIndex() = i;
    UiTestAccess::confirm();
    EXPECT_SCREEN(expected[i]);
  }
}

void test_cv_menu_row_mapping() {
  UiTestAccess::homeMode() = ModeSel::CV;
  const UiScreen expected[7] = {UiScreen::EDIT_SETPOINT_V, UiScreen::EDIT_SETPOINT_I, UiScreen::EDIT_TIMER,
                                UiScreen::EDIT_MAXTEMP, UiScreen::EDIT_OV, UiScreen::EDIT_OC, UiScreen::EDIT_OP};
  for (uint8_t i = 0; i < 7; i++) {
    UiTestAccess::screen() = UiScreen::SPEC_MENU;
    UiTestAccess::specMenuIndex() = i;
    UiTestAccess::confirm();
    EXPECT_SCREEN(expected[i]);
  }
}

// ---------------- starting a run ----------------

void test_run_test_row_starts_cc() {
  UiTestAccess::homeMode() = ModeSel::CC;
  UiTestAccess::screen() = UiScreen::SPEC_MENU;
  UiTestAccess::specMenuIndex() = UiTestAccess::specMenuRowCount() - 1;
  g_millis = 1234;
  UiTestAccess::confirm();
  EXPECT_STATE(LoadState::RUNNING_CC);
  TEST_ASSERT_EQUAL_UINT32(1234, testStartMs);
  TEST_ASSERT_EQUAL_UINT32(0, testTimeElapsed);
}

void test_run_test_row_starts_cv() {
  UiTestAccess::homeMode() = ModeSel::CV;
  UiTestAccess::screen() = UiScreen::SPEC_MENU;
  UiTestAccess::specMenuIndex() = UiTestAccess::specMenuRowCount() - 1;
  UiTestAccess::confirm();
  EXPECT_STATE(LoadState::RUNNING_CV);
}

void test_run_test_blocked_while_faulted() {
  UiTestAccess::homeMode() = ModeSel::CC;
  UiTestAccess::screen() = UiScreen::SPEC_MENU;
  UiTestAccess::specMenuIndex() = UiTestAccess::specMenuRowCount() - 1;
  faultActive = true;
  UiTestAccess::confirm();
  EXPECT_STATE(LoadState::IDLE);
  faultActive = false;
  loadState = LoadState::FAULT;
  UiTestAccess::confirm();
  EXPECT_STATE(LoadState::FAULT);
}

// ---------------- RUN screen ----------------

void test_run_pages_wrap() {
  UiTestAccess::screen() = UiScreen::RUN;
  UiTestAccess::runPage() = 0;
  UiTestAccess::rotate(-1);
  TEST_ASSERT_EQUAL_INT(2, UiTestAccess::runPage());
  UiTestAccess::rotate(1);
  TEST_ASSERT_EQUAL_INT(0, UiTestAccess::runPage());
}

void test_run_confirm_does_nothing() {
  UiTestAccess::screen() = UiScreen::RUN;
  loadState = LoadState::RUNNING_CC;
  UiTestAccess::confirm();
  EXPECT_SCREEN(UiScreen::RUN);
  EXPECT_STATE(LoadState::RUNNING_CC);
}

void test_abort_dialog_flow() {
  UiTestAccess::screen() = UiScreen::RUN;
  loadState = LoadState::RUNNING_CC;
  UiTestAccess::back();
  EXPECT_SCREEN(UiScreen::RUN_ABORT_CONFIRM);
  UiTestAccess::rotate(5);  // rotation ignored in the dialog
  EXPECT_SCREEN(UiScreen::RUN_ABORT_CONFIRM);
  UiTestAccess::back();  // "No, resume"
  EXPECT_SCREEN(UiScreen::RUN);
  EXPECT_STATE(LoadState::RUNNING_CC);
  UiTestAccess::back();
  UiTestAccess::confirm();  // "Yes, abort"
  TEST_ASSERT_TRUE(UiTestAccess::abortRequested());
  EXPECT_STATE(LoadState::IDLE);
}

// ---------------- handleGlobalRunEdge / summaries ----------------

static void startRun() {
  UiTestAccess::prevLoadState() = LoadState::IDLE;
  loadState = LoadState::RUNNING_CC;
  UiTestAccess::runEdge();
}

void test_run_edge_start_shows_run_screen() {
  UiTestAccess::runPage() = 2;
  startRun();
  EXPECT_SCREEN(UiScreen::RUN);
  TEST_ASSERT_EQUAL_INT(0, UiTestAccess::runPage());
}

void test_run_edge_completed() {
  startRun();
  loadState = LoadState::IDLE;
  UiTestAccess::runEdge();
  EXPECT_SCREEN(UiScreen::SUMMARY);
  TEST_ASSERT_EQUAL_INT((int)TestResult::COMPLETED, (int)UiTestAccess::lastSummary().result);
  TEST_ASSERT_EQUAL_UINT32(1, UiTestAccess::lastSummary().testIndex);
}

void test_run_edge_aborted() {
  startRun();
  UiTestAccess::abortRequested() = true;
  loadState = LoadState::IDLE;
  UiTestAccess::runEdge();
  TEST_ASSERT_EQUAL_INT((int)TestResult::ABORTED, (int)UiTestAccess::lastSummary().result);
  TEST_ASSERT_FALSE(UiTestAccess::abortRequested());  // cleared for the next run
}

void test_run_edge_fault_mid_run_beats_abort() {
  startRun();
  UiTestAccess::abortRequested() = true;
  faultReason = FaultReason::POWER;
  loadState = LoadState::FAULT;
  UiTestAccess::runEdge();
  TEST_ASSERT_EQUAL_INT((int)TestResult::FAULT, (int)UiTestAccess::lastSummary().result);
  TEST_ASSERT_EQUAL_INT((int)FaultReason::POWER, (int)UiTestAccess::lastSummary().fault);
}

void test_run_edge_fault_while_idle_reported_once() {
  UiTestAccess::prevLoadState() = LoadState::IDLE;
  faultReason = FaultReason::EXT_TEMP;
  loadState = LoadState::FAULT;
  UiTestAccess::runEdge();
  EXPECT_SCREEN(UiScreen::SUMMARY);
  TEST_ASSERT_EQUAL_UINT32(1, UiTestAccess::lastSummary().testIndex);

  UiTestAccess::screen() = UiScreen::HOME;
  UiTestAccess::runEdge();  // still faulted: must not create another summary
  EXPECT_SCREEN(UiScreen::HOME);
  TEST_ASSERT_EQUAL_UINT32(1, UiTestAccess::lastSummary().testIndex);
}

void test_summary_statistics() {
  startRun();
  const float volts[3] = {10.0f, 12.0f, 14.0f};
  const float amps[3]  = {1.0f, 2.0f, 3.0f};
  for (int i = 0; i < 3; i++) {
    measV_Bus = volts[i];
    totalCurrent = amps[i];
    measTemp_Int[0] = 30.0f + i;
    measTemp_Ext[0] = 20.0f + 2 * i;
    ui.recordStatsSample();
  }
  g_millis = 5000;
  loadState = LoadState::IDLE;
  UiTestAccess::runEdge();
  const TestSummary& s = UiTestAccess::lastSummary();
  NEAR(10.0f, s.minVoltage); NEAR(12.0f, s.avgVoltage); NEAR(14.0f, s.maxVoltage);
  NEAR(1.0f, s.minCurrent);  NEAR(2.0f, s.avgCurrent);  NEAR(3.0f, s.maxCurrent);
  NEAR(32.0f, s.maxIntTemp); NEAR(24.0f, s.maxExtTemp);
  TEST_ASSERT_EQUAL_UINT32(5000, s.durationMs);
}

void test_stats_ignored_when_not_running() {
  startRun();
  loadState = LoadState::IDLE;
  measV_Bus = 99.0f;
  ui.recordStatsSample();
  UiTestAccess::runEdge();
  NEAR(0.0f, UiTestAccess::lastSummary().maxVoltage);  // no samples taken, no div-by-zero
  NEAR(0.0f, UiTestAccess::lastSummary().avgVoltage);
}

// ---------------- persistence ring buffer ----------------

static TestSummary makeSummary(uint32_t idx) {
  TestSummary s;
  s.testIndex = idx;
  return s;
}

void test_recent_list_empty() {
  TestSummary out[5];
  TEST_ASSERT_EQUAL_INT(0, UiTestAccess::loadRecent(out));
}

void test_recent_list_partial_newest_first() {
  for (uint32_t i = 1; i <= 3; i++) UiTestAccess::persist(makeSummary(i));
  TestSummary out[5];
  TEST_ASSERT_EQUAL_INT(3, UiTestAccess::loadRecent(out));
  TEST_ASSERT_EQUAL_UINT32(3, out[0].testIndex);
  TEST_ASSERT_EQUAL_UINT32(2, out[1].testIndex);
  TEST_ASSERT_EQUAL_UINT32(1, out[2].testIndex);
}

void test_recent_list_ring_wraps_keeps_newest_five() {
  for (uint32_t i = 1; i <= 7; i++) UiTestAccess::persist(makeSummary(i));
  TestSummary out[5];
  TEST_ASSERT_EQUAL_INT(5, UiTestAccess::loadRecent(out));
  for (uint32_t i = 0; i < 5; i++) TEST_ASSERT_EQUAL_UINT32(7 - i, out[i].testIndex);
}

void test_recent_list_ring_exactly_full_and_boundary() {
  for (uint32_t i = 1; i <= 5; i++) UiTestAccess::persist(makeSummary(i));
  TestSummary out[5];
  TEST_ASSERT_EQUAL_INT(5, UiTestAccess::loadRecent(out));
  for (uint32_t i = 0; i < 5; i++) TEST_ASSERT_EQUAL_UINT32(5 - i, out[i].testIndex);
}

void test_recent_list_navigation() {
  for (uint32_t i = 1; i <= 3; i++) UiTestAccess::persist(makeSummary(i));
  UiTestAccess::homeRow() = 2;
  UiTestAccess::confirm();
  EXPECT_SCREEN(UiScreen::RECENT_LIST);
  TEST_ASSERT_EQUAL_INT(3, UiTestAccess::recentCount());
  spin(100);
  TEST_ASSERT_EQUAL_INT(2, UiTestAccess::recentListIndex());
  UiTestAccess::confirm();
  EXPECT_SCREEN(UiScreen::RECENT_DETAIL);
  UiTestAccess::rotate(100);
  TEST_ASSERT_EQUAL_INT(1, UiTestAccess::summaryScroll());  // clamps at 1
  UiTestAccess::back();
  EXPECT_SCREEN(UiScreen::RECENT_LIST);
  UiTestAccess::back();
  EXPECT_SCREEN(UiScreen::HOME);
}

void test_summary_screen_back_and_scroll() {
  UiTestAccess::screen() = UiScreen::SUMMARY;
  UiTestAccess::rotate(-5);
  TEST_ASSERT_EQUAL_INT(0, UiTestAccess::summaryScroll());
  UiTestAccess::rotate(5);
  TEST_ASSERT_EQUAL_INT(1, UiTestAccess::summaryScroll());
  UiTestAccess::back();
  EXPECT_SCREEN(UiScreen::HOME);
}

// ---------------- tick(): pins -> debounce -> navigation ----------------

void test_tick_confirm_press_fires_once() {
  g_millis = 1000;
  ui.tick();
  g_pinLevel[PIN_CONFIRM] = HIGH;  // confirm is active-high
  ui.tick();
  g_millis = 1039;
  ui.tick();
  TEST_ASSERT_EQUAL_INT((int)ModeSel::CC, (int)UiTestAccess::homeMode());  // not stable yet
  g_millis = 1040;
  ui.tick();
  TEST_ASSERT_EQUAL_INT((int)ModeSel::CV, (int)UiTestAccess::homeMode());
  g_millis = 2000;
  ui.tick();  // still held
  TEST_ASSERT_EQUAL_INT((int)ModeSel::CV, (int)UiTestAccess::homeMode());
}

void test_tick_back_button_is_active_low() {
  UiTestAccess::screen() = UiScreen::SPEC_MENU;
  g_millis = 1000;
  ui.tick();
  g_pinLevel[PIN_BACK] = LOW;
  ui.tick();
  g_millis = 1040;
  ui.tick();
  EXPECT_SCREEN(UiScreen::HOME);
}

void test_tick_idle_buttons_do_nothing() {
  for (uint32_t t = 0; t < 1000; t += 50) {
    g_millis = t;
    ui.tick();
  }
  EXPECT_SCREEN(UiScreen::HOME);
  TEST_ASSERT_EQUAL_INT((int)ModeSel::CC, (int)UiTestAccess::homeMode());
}

void test_tick_renders_every_call() {
  int before = display.frames;
  ui.tick();
  ui.tick();
  TEST_ASSERT_EQUAL_INT(before + 2, display.frames);
}

void test_resolution_full_range_without_drift() {
  UiTestAccess::enterEdit(UiScreen::EDIT_SETPOINT_I, 0);
  for (int i = 1; i <= 3000; ++i) {
    UiTestAccess::rotate(1);
    NEAR(i * 0.01f, UiTestAccess::editBuffer());
  }
  for (int i = 2999; i >= 0; --i) {
    UiTestAccess::rotate(-1);
    NEAR(i * 0.01f, UiTestAccess::editBuffer());
  }
  UiTestAccess::enterEdit(UiScreen::EDIT_SETPOINT_V, 0);
  for (int i = 1; i <= 300; ++i) {
    UiTestAccess::rotate(1);
    NEAR(i * 0.1f, UiTestAccess::editBuffer());
  }
}

void test_fine_current_save_and_screen_fit() {
  UiTestAccess::enterEdit(UiScreen::EDIT_SETPOINT_I, 0.05f);
  UiTestAccess::rotate(1);
  ui.tick();
  TEST_ASSERT_EQUAL_STRING("Step: 0.01 A (10 mA)", display.rows[3].c_str());
  for (const auto& row : display.rows) TEST_ASSERT_TRUE(row.size() <= 21);
  UiTestAccess::confirm();
  NEAR(0.06f, setCurrent);
  UiTestAccess::enterEdit(UiScreen::EDIT_SETPOINT_I, setCurrent);
  UiTestAccess::rotate(1);
  UiTestAccess::back();
  NEAR(0.06f, setCurrent);
}

void test_entry_clamps_old_out_of_range_values() {
  UiTestAccess::enterEdit(UiScreen::EDIT_SETPOINT_V, 150);
  UiTestAccess::confirm();
  NEAR(30, setVoltage);
  UiTestAccess::enterEdit(UiScreen::EDIT_SETPOINT_I, 30.3f);
  UiTestAccess::confirm();
  NEAR(30, setCurrent);
}

void test_current_step_toggle_preserves_value() {
  setCurrent = 0.06f;
  for (int step = 1; step <= 3; ++step) {
    UiTestAccess::screen() = UiScreen::SPEC_MENU;
    UiTestAccess::specMenuIndex() = UiTestAccess::specMenuRowCount() - 2;
    UiTestAccess::confirm();
    EXPECT_STATE(LoadState::IDLE);
    NEAR(0.06f, setCurrent);
    UiTestAccess::enterEdit(UiScreen::EDIT_SETPOINT_I, setCurrent);
    UiTestAccess::rotate(1);
    NEAR(step == 1 ? 0.16f : (step == 2 ? 1.06f : 0.07f), UiTestAccess::editBuffer());
    UiTestAccess::back();
  }
}
void test_power_ceiling_and_invalid_inputs() {
  NEAR(500, LoadSetpoints::powerLimit(900));
  NEAR(0, LoadSetpoints::powerLimit(NAN));
  NEAR(500.0f/30, LoadSetpoints::currentCeiling(30,900,true));
  NEAR(30, LoadSetpoints::currentCeiling(5,500,true));
  NEAR(10, LoadSetpoints::currentCeiling(30,300,true));
  NEAR(0, LoadSetpoints::currentCeiling(30,500,false));
  NEAR(0, LoadSetpoints::currentCeiling(NAN,500,true));
  NEAR(0, LoadSetpoints::currentCeiling(-1,500,true));
  NEAR(0, LoadSetpoints::currentCeiling(0,0,true));
  for (int v=1; v<=300; ++v) {
    float voltage=v/10.0f;
    TEST_ASSERT_TRUE(LoadSetpoints::currentCeiling(voltage,500,true)*voltage <= 500.0001f);
  }
}
void test_timer_hours_seconds_preserve_duration() {
  setTime=10;
  UiTestAccess::screen()=UiScreen::SPEC_MENU;
  UiTestAccess::specMenuIndex()=UiTestAccess::specMenuRowCount()-3;
  UiTestAccess::confirm();
  NEAR(10,setTime);
  UiTestAccess::enterEdit(UiScreen::EDIT_TIMER,setTime);
  UiTestAccess::rotate(1);
  NEAR(3610,UiTestAccess::editBuffer());
  ui.tick();
  TEST_ASSERT_EQUAL_STRING("01:00:10",display.rows[2].c_str());
  for(const auto& row:display.rows) TEST_ASSERT_TRUE(row.size()<=21);
  UiTestAccess::confirm();
  NEAR(3610,setTime);
  UiTestAccess::specMenuIndex()=UiTestAccess::specMenuRowCount()-3;
  UiTestAccess::confirm();
  UiTestAccess::enterEdit(UiScreen::EDIT_TIMER,setTime);
  UiTestAccess::rotate(1);
  NEAR(3615,UiTestAccess::editBuffer());
  UiTestAccess::back(); NEAR(3610,setTime);
  UiTestAccess::specMenuIndex()=UiTestAccess::specMenuRowCount()-3;
  UiTestAccess::confirm();
  UiTestAccess::enterEdit(UiScreen::EDIT_TIMER,0);
  UiTestAccess::rotate(24); NEAR(86400,UiTestAccess::editBuffer());
  UiTestAccess::rotate(1); NEAR(86400,UiTestAccess::editBuffer());
  UiTestAccess::rotate(-100); NEAR(0,UiTestAccess::editBuffer());
}
int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_timer_hours_seconds_preserve_duration);
  RUN_TEST(test_current_step_toggle_preserves_value);
  RUN_TEST(test_power_ceiling_and_invalid_inputs);
  RUN_TEST(test_resolution_full_range_without_drift);
  RUN_TEST(test_fine_current_save_and_screen_fit);
  RUN_TEST(test_entry_clamps_old_out_of_range_values);
  RUN_TEST(test_quadrature_directions);
  RUN_TEST(test_quadrature_full_table_properties);
  RUN_TEST(test_detents_carry_remainder);
  RUN_TEST(test_detents_negative);
  RUN_TEST(test_detents_back_and_forth_cancels);
  RUN_TEST(test_debounce_single_edge_after_stable);
  RUN_TEST(test_debounce_rejects_bounce);
  RUN_TEST(test_debounce_release_gives_no_edge_and_repress_works);
  RUN_TEST(test_debounce_handles_millis_rollover);
  RUN_TEST(test_home_rotate_clamps);
  RUN_TEST(test_menus_move_one_row_per_rotate_call);
  RUN_TEST(test_home_confirm_toggles_mode);
  RUN_TEST(test_home_to_spec_menu_and_back);
  RUN_TEST(test_home_to_recent_list_empty);
  RUN_TEST(test_spec_menu_row_count);
  RUN_TEST(test_spec_menu_rotate_clamps_and_scrolls);
  RUN_TEST(test_edit_screens_step_clamp_save);
  RUN_TEST(test_edit_back_cancels_without_saving);
  RUN_TEST(test_cc_menu_row_mapping);
  RUN_TEST(test_cv_menu_row_mapping);
  RUN_TEST(test_run_test_row_starts_cc);
  RUN_TEST(test_run_test_row_starts_cv);
  RUN_TEST(test_run_test_blocked_while_faulted);
  RUN_TEST(test_run_pages_wrap);
  RUN_TEST(test_run_confirm_does_nothing);
  RUN_TEST(test_abort_dialog_flow);
  RUN_TEST(test_run_edge_start_shows_run_screen);
  RUN_TEST(test_run_edge_completed);
  RUN_TEST(test_run_edge_aborted);
  RUN_TEST(test_run_edge_fault_mid_run_beats_abort);
  RUN_TEST(test_run_edge_fault_while_idle_reported_once);
  RUN_TEST(test_summary_statistics);
  RUN_TEST(test_stats_ignored_when_not_running);
  RUN_TEST(test_recent_list_empty);
  RUN_TEST(test_recent_list_partial_newest_first);
  RUN_TEST(test_recent_list_ring_wraps_keeps_newest_five);
  RUN_TEST(test_recent_list_ring_exactly_full_and_boundary);
  RUN_TEST(test_recent_list_navigation);
  RUN_TEST(test_summary_screen_back_and_scroll);
  RUN_TEST(test_tick_confirm_press_fires_once);
  RUN_TEST(test_tick_back_button_is_active_low);
  RUN_TEST(test_tick_idle_buttons_do_nothing);
  RUN_TEST(test_tick_renders_every_call);
  return UNITY_END();
}

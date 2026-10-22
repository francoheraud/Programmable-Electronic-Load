# Tests

- `test_ui_logic/` – host-side Unity tests for `lib/UI` (quadrature decode, detent carry, button debounce,
  menu/edit/run/summary state machine, NVS ring buffer, `tick()` with simulated pins).
- `support/` – stand-ins for `Arduino.h`, `Preferences.h` and `Display.h`, used only by the `native` env.

Run the UI tests on your PC (no board needed; requires a host `g++`, e.g. MinGW-w64 on Windows):

    pio test -e native

`-DUNIT_TEST` makes `UiController` declare `friend struct UiTestAccess`, which is how the tests reach its
private state. It is not defined for the firmware builds.

Hardware-bound modules (INA, ADS7128, ADS1015, DAC, SdLog, Display) are verified on the bench, not here.

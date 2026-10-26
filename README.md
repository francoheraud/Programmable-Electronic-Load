# Programmable Electronic Load
ELEC5552 Electrical and Electronic Engineering Design Project - Team B04

## Demonstration firmware (ESP32-S3 N16R8)

Build the real electronic-load application explicitly; the default environment is the simulated UI bring-up build.

```powershell
.\pio.ps1 run -e load_n16r8
.\pio.ps1 run -e load_n16r8 -t upload --upload-port COM3
.\pio.ps1 device monitor -e load_n16r8 -p COM3 -b 115200
```

Use `load_n16r8_usb` for native USB instead of the USB-UART bridge. The PowerShell helper uses the local Python 3.12 and PlatformIO installation; a normal PlatformIO installation can run the equivalent `pio` commands directly.

This branch integrates the bench-developed firmware from `lachy-update`: corrected ADC reads and DAC scaling, initialized protection thresholds, bounded CV PI control, asynchronous SDHC logging, optional external thermistors, serial diagnostics, and the final OLED controls. PCB pin assignments are retained.

- CV trial settings: Kp=0.002, Ki=0.008, Kd=0, 50 ms sampling, with slew limiting and anti-windup.
- Current steps: 10 mA / 100 mA / 1 A; voltage steps: 0.1 V.
- Timer: seconds/hours adjustment, HH:MM:SS display, maximum 24 hours.
- Display: permanently rotated 180 degrees; running temperature page shows six internal and four external sensors.
- `temps`, `health`, and `help` are available in the serial monitor.
- External probes are optional; unavailable readings are shown as `--` and logged as `nan`. Internal sensor protection and ADC communication checks remain active.
- Command limits are 30 V, 30 A and 500 W, with voltage-based current limiting and measured-power trips. Software cannot guarantee zero transient overshoot or establish hardware qualification at those ratings.
- CC uses the corrected nominal shunt/divider/DAC conversion. No fitted supply-current slope/offset correction is implemented.

The latest source passed both full firmware builds and 49 host UI/control-helper tests before publication. Hardware performance still requires bench validation; see the test folders and commissioning notes for earlier investigations. Historical notes may describe superseded settings.

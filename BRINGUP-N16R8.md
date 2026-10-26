# OLED and controls bring-up (custom PCB, N16R8)

Imported from francoheraud/Programmable-Electronic-Load, lachy-update,
commit deb42ead5d91f50b1e0d62aa39495f3e78230d8b.

Default environment: ui_n16r8 (USB-to-UART adapter / bridge).
For the ESP32-S3 native USB connector, use ui_n16r8_usb instead.
Both configure 16 MB flash, 8 MB octal PSRAM and a 16 MB partition table.
The original upstream environments are retained for reference; they are not
validated by this integration and should not be selected for this UI test.

The default test uses the branch's real UI, OLED, encoder decoder and debounce
logic. Sensor and actuator commands are excluded from this build. UI test
voltage/current/temperature values are SIMULATED. Keep the load input supply
disconnected during these UI checks: this firmware does not control the power stage.

## PCB connections retained

- SSD1306 128x64 I2C: SDA GPIO17, SCL GPIO18, address 0x3C.
- Encoder phase A GPIO38, phase B GPIO39 (internal pull-ups).
- Encoder push GPIO40: pressed HIGH (internal pull-down).
- Back button GPIO14: pressed LOW (internal pull-up).
- All signal levels are 3.3 V.

## PowerShell commands

Run from this project folder. The helper uses your working Python 3.12 because the existing platformio.exe launcher points to an unavailable Python installation:

```powershell
.\pio.ps1 run -e ui_n16r8
.\pio.ps1 device list
.\pio.ps1 run -e ui_n16r8 -t upload --upload-port COM5
.\pio.ps1 device monitor -e ui_n16r8 -p COM5
```

Replace COM5 with the actual port. For native USB replace ui_n16r8 with
ui_n16r8_usb in all commands. Native USB may change COM number after flashing.
Close the serial monitor before uploading. Ctrl+C closes the monitor.

Type one command at a time in the serial monitor and press Enter:

1. `scan`: expect an ACK at 0x3C (or 0x3D if your address strap differs).
2. `oled`: eight text rows; repeat to display a second pattern.
3. `pins`: inspect input levels at rest, then while pressing each button.
4. `enc`: turn slowly both directions; step count changes sign, nominally four
   transitions per detent. `stop` exits.
5. `btn`: each press and release should generate an event after 40 ms debounce.
6. `ui`: exercise the real menu with encoder, confirm and back. Set Timer > 0
   before trying a simulated Run Test. `stop` exits.

Display initialization now explicitly checks I2C acknowledgement before declaring
success and preserves setup's I2C pins. If scan sees 0x3D, adjust OLED_I2C_ADDR
in lib/Display/Display.h, rebuild and upload. A responding I2C address does not
identify the controller: if the image stays blank/offset, verify the module is
SSD1306 rather than SH1106 (diagonal size alone does not identify it).

Nothing is uploaded automatically. Compilation cannot verify physical wiring,
button polarity, display controller or power-stage isolation.


## Validation status

The installed files were compared against the prepared integration and match.
PlatformIO successfully parsed both N16R8 environments and their inherited
settings. COM3 was detected as a CH343 USB-to-serial adapter; confirm this is your
board before uploading.

Compilation is NOT verified: both installed and freshly downloaded Xtensa
compilers fail to launch in this Codex session with Windows error
"Illegal System DLL Relocation". Run the build command in a normal PowerShell
terminal to determine whether this is specific to this execution environment.
No firmware has been uploaded and no physical hardware tests were performed.

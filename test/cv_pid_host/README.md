# CV controller: tuning revision 2

Bench observation: v1 repeatedly crossed the supply CV/CC transition and undershot
4 V to 2.98 V. This revision is a conservative retune, NOT hardware-validated stability.

Defaults in include/CvPid.h:
Kp=0.002 A/V, Ki=0.005 A/(V*s), Kd=0 (PI operation).
Filtered derivative remains implemented (tau 0.300 s) but is disabled pending
waveform-based tuning. Current slew: +0.010 / -0.030 A/s.
CV acquisition schedule: 50 ms instead of 200 ms. Actual cadence may be slower
because of UI/I2C work; measured elapsed time is used, not an assumed fixed period.
CC/idle acquisitions remain at 200 ms. INA failure timeout remains 400 ms.

Only new successful INA acquisitions advance the controller. Output tracking
anti-windup covers current and slew limits. Stop/fault, failed/stale measurements
and excessive time gaps reset to zero. Reduced current ceilings act immediately.
No pin, UI, CC DAC scaling, INA calibration or protection thresholds changed.

Host regression checks (from project root):
  g++ -std=c++14 -Wall -Wextra -Iinclude test/cv_pid_host/test.cpp -o cv_pid_test.exe
  ./cv_pid_test.exe
Checks include time scaling, fresh-sample handling, current/slew limits, anti-windup,
stop/failure resets, millis wraparound, optional derivative and the 50 ms cadence.
These checks do not establish stability of the actual hardware loops.

Bench: supply 5 V / 0.10 A limit; e-load CV 4 V / 0.20 A ceiling; 30 seconds.
Allow roughly 15-20 seconds for the deliberately slow approach. Source begins in
CV and should enter CC as the load lowers voltage. Stop if repeated collapse or
large oscillations persist; do not raise current limits to suppress oscillations.
Record the live voltage range during the final seconds (not just whole-run average),
then verify return to idle. A scope/captured waveform is needed if cycling persists.

V3 TRIAL supersedes the default gains above: Ki=0.008 and rise=0.015 A/s.
Kp=0.002, Kd=0, 50 ms acquisition and fall=0.030 A/s remain unchanged.
See Integration_test_plan.txt for comparisons and staged tests. V2 preserved in backups.

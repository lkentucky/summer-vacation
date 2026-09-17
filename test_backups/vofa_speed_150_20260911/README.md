# Temporary 150 cm/s wheel-speed test

## Status: normal source restored on 2026-09-11

The temporary test, UART1 PID parser and JustFloat telemetry have been removed
from the active project at the user's request. The four modified source/header
files were verified against the pre-test working-copy backups. The previous
test firmware, source files, configuration and host tests are preserved in
`test_firmware_before_restore/`; the instructions below describe that archived
test version, not the currently active normal firmware.

No flashing was performed during restoration. The source speed PID remains
P=9.36, I=0.5, D=0.01. The user has not yet supplied the final values tuned in
RAM over UART; obtain them before replacing the running test firmware if they
are to be retained. Changing PC source does not itself erase those RAM values.

The four adjacent backup files are the working copies from before this test,
including the user's uncommitted tuning and camera changes. Do not use a Git
reset/checkout to restore them. If later edits are made, remove only the test
changes instead of overwriting newer work.

## Operation

- Raise both drive wheels and secure the car before powering the motor driver.
- Double-click KEY4 (existing approximately 400 ms window) to start both wheels
  at a target of 150 cm/s using the existing 2 ms speed PID.
- Double-click KEY4 again to stop. A 10-second TIM6 timeout also stops the motors.
- Power-on never starts the motors. Keep the hardware power switch accessible.
- Camera/IMU initialization, image steering, speed tiers, image-based stops and
  Bluetooth control are bypassed in this bench-only build. Do not run it on track.
- UART1 TX=A9, RX=A10; use a 3.3 V TTL USB-UART adapter, connect A9 to adapter RX,
  adapter TX to A10, and share GND. Do not use RS-232 voltage levels or join power
  supplies blindly.
- VOFA+: Serial, 115200 baud, 8 data bits, no parity, 1 stop bit, no flow control;
  select JustFloat. Frames are sent every 20 ms, including while stopped.
- CH0: left encoder-derived speed, cm/s. CH1: right encoder-derived speed, cm/s.
  CH2: shared left/right target, cm/s. CH3/CH4: signed left/right PWM command,
  -5000..5000 in the existing controller (10000 means full scale).
  CH5/CH6/CH7: current speed Kp/Ki/Kd. CH8: last command result, 0=none,
  1=applied, -1=syntax, -2=range/nonfinite, -3=invalid byte/overflow, -4=timeout.
- Encoder scale is unchanged: D=6.5 cm, PPR=1024, gearing factor 30/68;
  left count sign is negated, right is positive. These are estimated wheel
  peripheral speeds, not independent ground-speed measurements. Physical encoder
  resolution, gear ratio, diameter and polarity still require calibration.
- JustFloat: nine little-endian float32 values + 00 00 80 7F, 40 bytes per frame.
  Official format: https://www.vofa.plus/docs/learning/dataengines/justfloat/

## One-command speed PID adjustment (requires the updated firmware)

Send ASCII/text, not hexadecimal, on the same UART1 connection:

```text
PID 9.36 0.5 0.01
```

Append a real CR, LF or CRLF terminator. The order is Kp Ki Kd; accepted ranges
are 0..100, 0..20, 0..20. The whole line is validated before all three values
are updated in a short critical section. Lowercase `pid` is also accepted.
The UART interrupt only buffers bytes; parsing and telemetry run in the main
loop. Incomplete lines time out after one second; overflowed lines are discarded.
The update does not start/restart motors or extend the ten-second deadline.
The current accumulated PWM and error history are retained, not reset mid-run.
Values are RAM-only; reboot restores the compiled defaults. Only speed-loop
gains are changed, not vision/yaw gains. Start tuning with wheels secured off
the ground. No ASCII acknowledgements are mixed into the binary waveform stream;
check CH5..CH8 for confirmation. CH8 retains the result of the latest command.

## Restoration after the physical test

Set MOTOR_SPEED_TEST_ENABLE to 0 in
smartcarmain/user/inc/motor_speed_test.h, then rebuild and flash the normal firmware.
This restores normal camera/IMU/Bluetooth/KEY4 control paths and removes test
telemetry. For an exact source cleanup, reverse the guarded additions in motor.h,
motor.c, isr.c and main.c while retaining all other user edits. The configuration
header and host tests can then be removed. Changing PC source alone does not
restore firmware already on the board.

This task prepares and builds firmware; physical testing and flashing are not
performed automatically. Tell the assistant when the test is finished to restore.

## Verification completed

- ArmClang 6.16 syntax checks passed for motor.c, isr.c, main.c and mymenu.c
  with MOTOR_SPEED_TEST_ENABLE=1 and with it set to 0.
- Host tests of the actual motor.c passed: idle motor-off, encoder conversion,
  fixed equal targets, start/stop, preserved PID output limit, ten-second timeout,
  tick wraparound, interrupt-mask preservation, JustFloat payload and send rate.
- Additional PID command host tests passed: CR/LF/CRLF, fragmented input,
  missing/extra tokens, nonfinite/out-of-range values, line and ring-buffer
  overflow recovery, timeout recovery, preserved run/deadline state and PID echo.
- Full Keil test-mode build passed: 0 errors, 0 warnings (build_test_retry.log).
  The first sandboxed attempt failed writing the assembler output; the authorized
  normal build succeeded. No flashing was performed.
- Original working-copy whitespace in motor.h was retained rather than changed.

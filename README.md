# ROV Motor Test

A small browser UI for testing the eight thrusters of an ArduSub vehicle from
the onboard Linux computer (RK3588). Each click arms the flight controller,
runs one motor for a bounded time, then disarms.

The UI shows only motor numbers, throttle, duration, and the operator-recorded
physical position/direction. It does not change the flight controller mixer or
any parameters.

## Safety

**Remove the propellers or disconnect thruster power before using this tool.**
The tool arms the vehicle automatically. The `props off` checkbox is an
operator confirmation, not a hardware interlock.

- Use only on a trusted, isolated network. The HTTP server has no login and no
  TLS.
- Stop the ROV bridge/gateway before starting this tool. Only one process can
  own `/dev/ttyACM0`.
- The tool caps thrust at +/-40% and duration at 10 seconds per test, but those
  limits do not make an unsecured vehicle safe.
- This is a bench/setup tool, not a flight-control interface.

## Requirements

- Linux with Python 3.8+
- `pymavlink` and `pyserial`
- ArduSub flight controller connected on `/dev/ttyACM0` at 115200 baud

## Install and run

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
python3 motor_web.py
```

Open:

```text
http://<vehicle-ip>:8088/
```

Stop the ROV bridge/gateway first using its normal service or process command.
Only one process can own `/dev/ttyACM0`.

Check the serial connection:

```bash
curl http://127.0.0.1:8088/api/status
```

A healthy response contains `"serial_ok": true`. Logs go to stdout; when using
`nohup`:

```bash
nohup python3 motor_web.py >/tmp/motor_web.log 2>&1 &
```

Stop the tool:

```bash
pkill -f '^python3 .*motor_web.py$'
```

The tool is not installed as a systemd service, so restart it after a reboot.

## Usage

1. Remove propellers or disconnect thruster power.
2. Open the web page and confirm the `props off` checkbox.
3. Set thrust (default +20%, 0 = stop, PWM = 1500 + thrust*4) and duration.
   The PWM column in the live table is the ground truth; ArduPilot's motor-test
   percent scale is *not* a thrust percentage.
4. Click `M1` ... `M8` one at a time.
5. Record the actual position and direction in the table.
6. Use `Export JSON` to save the mapping.

The mapping table is stored in the browser's `localStorage`; it is not uploaded
anywhere. Clearing site data or using another browser loses it.

## Mount and lights

`云台舵机` drives AUX1 through `MAV_CMD_DO_MOUNT_CONTROL` (mode
`MAV_MOUNT_MODE_MAVLINK_TARGETING`). It only works when `SERVO9_FUNCTION=7`
(`mount_tilt`); the page warns at connect if it is anything else. The pitch
slider range comes from `MNT1_PITCH_MIN/MAX` (this vehicle: -90..+20), so
0 deg maps to ~1754, not 1500.

`灯` drives AUX2 through `MANUAL_CONTROL.buttons`, which ArduSub routes to the
`BTNx_FUNCTION` joystick handlers. On this vehicle `BTN14_FUNCTION=32`
(brighter) and `BTN13_FUNCTION=33` (dimmer). Those bits are discovered at
connect; the page warns if none is mapped. `DO_SET_SERVO` cannot be used here
because AUX2 is owned by `lights1`.

## HTTP API

`POST /api/test`

```json
{"motor": 1, "thrust": 20, "seconds": 3, "props_off": true}
```

`POST /api/mount` takes `{"pitch": -90}` (degrees).

`POST /api/light` takes `{"name": "brighter"|"dimmer"|"cycle", "repeat": 1..12}`.

`POST /api/stop` stops the current test and disarms. `POST /api/disarm` also
disarms. `GET /api/status` returns the current state.

The server rejects `props_off` values other than `true`, motor numbers outside
1..8, `thrust` outside -40..40 or equal to 0, and duration above 10 seconds.
Internally the tool sends ArduPilot motor-test percent, which the firmware maps
linearly onto RC3_MIN..MAX (1100..1900): percent 50 is neutral (PWM 1500) and
percent 5 would be PWM 1140, almost full reverse.

## Live output check

`SERVO_OUTPUT_RAW` is streamed at 10 Hz and shown as M1..M8. If the commanded
channel does not reach 1500 + thrust*4 within 1.5 s, the test aborts and
disarms with `Mx 无响应` -- the FC accepting arm but ignoring the motor test
has been seen on the bench, and without this check a dead test looks like a
successful one.

## Motor numbering and ArduSub workaround

`MAV_CMD_DO_MOTOR_TEST.param1` is specified as a 1-based motor number (1..8).
The ArduSub version used here passes that value directly to
`AP_MotorsMatrix::output_test_num()`, which indexes motors from 0. As a result:

- `param1 = 8` is rejected, so the last motor cannot be tested.
- `param1 = 1` drives physical motor 2, and every channel is shifted by one.

This tool compensates by sending `motor - 1` in `Tester._motor_cmd()`. Keep
the UI labels as 1..8; do not change them to 0..7.

If the flight-controller firmware is fixed later, remove the `- 1` in
`_motor_cmd()` at the same time. Otherwise the tool will be off by one again.
QGC's own Motor Test remains affected by the firmware bug until it is fixed.

This workaround affects motor testing only. It does not affect normal thruster
mixing, manual control, or depth control.

## Verification

The tool was bench-tested on:

- RK3588 running Linux, Pixhawk1 on `/dev/ttyACM0`
- ArduSub with `FRAME_CONFIG=2` (8 thrusters), `SERVO1..8_FUNCTION=33..40`
- Motors 1 and 2 exercised with propellers removed
- M8 confirmed via `SERVO_OUTPUT_RAW`: the corrected request drives output
  channel 8, while the raw 1-based request is rejected

The physical mapping recorded before the numbering fix is invalid and must be
re-recorded with this build.

No automated tests are included because the program requires a live MAVLink
serial connection and an armed flight controller.

## Files

```text
motor_web.py       HTTP server, MAVLink state machine, embedded UI
requirements.txt   Python dependencies
```

## License

No license has been granted. All rights reserved unless a license is added.

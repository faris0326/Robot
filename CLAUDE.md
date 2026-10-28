# Line Follower Robot (ESP32-S3) — Project Brief

Speed-race line follower for an Industrial Design course. At the end of the semester all robots race on the same track and the fastest wins. Firmware runs on an ESP32-S3, with live telemetry to a phone and automatic lap timing.

## Goals (priority order)
1. Robot follows the line reliably and fast.
2. Smooth live telemetry on a phone with no external server.
3. Automatic lap timing measured on the robot.
4. Code structured so the sensor array and PID can be upgraded later without rewrites.

## Hardware
- **MCU:** YD ESP32-S3 (N16R8)
- **Motor driver:** DRV8833 (J2 jumpered, motors confirmed working)
- **Motors:** N20, **rated 3 V** (new motors with a different reduction, 2026-10-21; encoders no longer used). Driven from the 5 V rail, so PWM is capped at 3/5 = 60 % (`MAX_DUTY` = `MOTOR_RATED_V / MOTOR_SUPPLY_V`).
- **Battery:** 2S 18650 Li-ion (8.4 V full), KCD1 rocker switch on battery +
- **Regulator:** MINI560 5 V buck
- **Current sensor:** INA219 (I2C) — **not used in code yet, not in the schematic**
- **Sensors:** **8-channel QTR-8A clone** (board "V2766", "8 sensor lines follow", pins VCC, D8..D1, IR, GND), in the code since 2026-10-21. **Confirmed analog**: each channel is a phototransistor with a 10 kΩ pull-up ("103" arrays) and a 220 Ω series resistor ("221" arrays), no comparators. At ~3 mm on 3.3 V: white 0.3 V, black 2.8 V. Its 3.3V pads are bridged, so it must stay on **3.3 V** (5 V would overdrive the LEDs). IR pin = LED enable, pulled on by R6 (47 kΩ). Replaced the 5-channel digital array (comparator + potentiometers). Possibly a custom 16-channel array later.
- **Track:** Cytron Reconfigurable Line Following Track, black line 18 mm wide
- **Build system:** PlatformIO (`platformio.ini`, espressif32@6.13.0 = Arduino core 2.0.17)

### Power (from schematic Sheet_1 rev 1.0)
A single +5 V rail (MINI560 output, through the rocker switch) feeds the ESP32 5V pin, DRV8833 VCC, the IR array, and both encoder VCCs. Motors therefore see at most 5 V, so `MAX_DUTY` limits speed, not overvoltage.

**Resolved (user, 2026-10-15):** the IR array and encoder outputs no longer drive the 3.3 V-only GPIOs with 5 V. The schematic (including the 2026-10-16 version) still shows the IR array and encoder VCC on +5 V.

**INA219 (schematic 2026-10-16):** VCC from the ESP32 3V3 pin, SDA 41 / SCL 42, shunt (Vin+ → Vin−) in series between the rocker switch and the +5 V rail. In that position it measures the **regulated 5 V rail and total robot current**, not the battery, so it can't give battery voltage or charge state. It can't be moved: the **BMS and MINI560 are mounted on the battery pack, off the board**; only 5 V reaches the board.

### Pin map (source of truth: the schematic; all pins in `include/config.h`)
| Function | GPIO |
|---|---|
| Right motor: DRV8833 IN1 / IN2 (OUT1/OUT2 → M1_R/M2_R) | 18 / 17 |
| Left motor: DRV8833 IN3 / IN4 (OUT3/OUT4 → M1_L/M2_L) | 16 / 15 |
| IR array D1..D8 (analog) | 1, 2, 4, 5, 6, 7, 8, 9 (`PIN_IR` lists them left → right: D8..D1) |
| IR array LED enable (IR pin) | 11 |
| I2C (INA219, address 0x40): SDA / SCL | 41 / 42 |

**IR sensor pins must be ADC1 (GPIO 1–10, not strapping pin 3).** ADC2 (GPIO 11–20) can't be read while Wi-Fi is on; `sensors.cpp` has a `static_assert` that fails the build otherwise. The user first wired D6–D8 to GPIO 12/13/14 and **rewired them to 7/8/9** (confirmed 2026-10-22). **D1 is the rightmost sensor** (looking from behind, confirmed with sensortest 2026-10-22), so `PIN_IR` runs D8..D1. Encoders are no longer wired.

**Motor direction (new 3 V motors, motortest 2026-10-22): neither motor is inverted** (`MOTOR_L_INVERT = false`, `MOTOR_R_INVERT = false`). The old motors needed the left one inverted.

History, old 5-channel digital array (sensortest, twice): O1 was the leftmost sensor. The old digital array output LOW on black (setting removed with it). Lesson kept: check sensor polarity with the in-air test (no reflection = black), not a guess.

Line colour is set by `LINE_IS_WHITE`: **`true` = white line on a dark surface** (user, 2026-10-22; set `false` for the black-line race track). Calibration measures the two readings per sensor and this setting says which one is the line (auto-detecting the colour from the spin was tried earlier the same day and replaced, so a wrong guess can't flip it). `sensorsSetCalibration()` swaps a sensor's readings if they're the wrong way round for the setting, so a saved calibration survives a colour change. Sensor bits always mean "on the line", whatever the colour.

### Next hardware: PCB as chassis (in development, schematic final 2026-10-24)
Schematic: `schematic/pcb_chassis_sheet1.png`. The PCB doubles as the chassis to save weight for high-rpm, low-torque motors. Plan: build it on a breadboard first, design the PCB once it works. **The firmware has not been updated for it yet** (user's choice: wait until the breadboard is ready). Differences from the current build:
- **DRV8833 VCC on the battery side** (`DRIVER` net = MINI560 IN+, after the INA219), on purpose: the driver handles it, and PWM is limited per motor's rated voltage. With the current `MOTOR_SUPPLY_V = 5` the 3 V motors would get ~4.4–5 V, so the firmware must base `MAX_DUTY` on the measured battery voltage before driving motors on this board.
- **INA219 between SW1 (MST-12D18G4 slide switch) and the battery-side node**, so it measures pack voltage and total current, motors included. `power.cpp` needs updating: no `BUCK_EFFICIENCY` division, battery-low threshold ~6.4 V instead of `RAIL_LOW_V` 4.75 V, % can use voltage.
- **IR LED enable on GPIO 10** (`PIN_IR_LED` is still 11).
- **Encoder connectors kept** (C2-R / C1-R / C1-L / C2-L on GPIO 11 / 12 / 13 / 14, encoder VCC +5 V) so the old encoder motors can be swapped back in. Check that encoder outputs don't reach 5 V before using them.
- DRV8833 `ULT` (nFAULT) left unconnected (user decision).

## Constraints and decisions
- **Analog sensor reading** (since 2026-10-21, with the QTR-8A clone). Digital reading was used first.
- **No PID at first.** Start with simple proportional control, then add D, then I only if needed.
- **No encoders in the code for now.** Leave clean stubs or hooks so they can be added later. (INA219 approved by the user and built 2026-10-16, see Battery estimate below.)
- **Motor voltage cap: 3 V.** All speeds (code, tuning, phone) are fractions of the rated motor voltage: 1.0 / 100 % = 3 V. `motors.cpp` multiplies by `MAX_DUTY` (0.6) when setting PWM, so nothing can exceed 3 V. Telemetry "duty" fields carry these speeds (after trim). Cruising speed is `BASE_SPEED`.
- **Boots in Stop** (`BOOT_MODE`, user, 2026-10-22; it used to start Auto by itself): the robot doesn't move after power-on or upload until a mode is picked on the phone. The motors also stay off for `START_DELAY_MS` (3 s) after boot in every mode.
- Spend no more money: use only the ESP32-S3 on the robot (no extra MCUs or computers).

## Architecture

### Dual-core split
- **Core 1: control loop.** Sensors, `readPosition()`, control, motor output, lap detection. Fixed period (start at 1–2 ms) using `micros()` or a hardware timer so `dt` is consistent.
- **Core 0: network.** Wi-Fi, web server, WebSocket, telemetry sending. Create the telemetry task with `xTaskCreatePinnedToCore(..., 0)`.
- If using ESPAsyncWebServer, make sure the AsyncTCP task runs on core 0 (check `CONFIG_ASYNC_TCP_RUNNING_CORE`), so it does not steal time from the control loop.
- The control loop must never call network functions and never wait on telemetry.
- Share data through a small struct that the control loop writes and the telemetry task copies (short critical section) or a length-1 overwrite queue. Always send the **latest** sample; drop old ones.

### Sensor abstraction
- All line-position logic lives behind `readPosition()`, which returns the line position as a number.
- Digital version: weighted average of the active sensors, e.g. weights `-4, -2, 0, +2, +4` for 5 sensors (adjacent pairs give in-between values).
- Include a **lost-line fallback**: when no sensor sees the line, keep turning toward the last known direction.
- **Analog version (built 2026-10-21, `src/sensors.cpp`):** 12-bit `analogRead` of 8 channels per tick. Each raw value maps through its calibration (background → 0, line → 1, either polarity) to a level 0..1. Position = level-weighted average of weights −7…+7 (levels < `IR_NOISE_FLOOR` ignored), scaled to ±1. Lost when no level > `IR_ON_LINE`; it then returns ±1 toward the last side (`lastSide`, see Sharp corners). The analog array never reports `InGap` (that was for the old digital array, whose line could sit between sensors).
- **Calibration (reworked 2026-10-22, so the array height doesn't matter):** phone "Calibrate" button = mode `Calibrate` (3). The control task spins right/left/right for `CAL_DURATION_MS` at `CAL_SPIN_SPEED` while `readPosition()` fills a histogram per sensor (32 raw counts per bin), then switches to Stop. `sensorsCalibrateEnd()` splits each histogram into two groups with Otsu's method (each group ≥ 1 % of samples, so stray readings can't pose as the line) and stores the group **averages** as `background` / `line` (not min/max, which one stray reading skews). On-line threshold = level 0.5 = halfway between them. `LINE_IS_WHITE` says which group is the line (raw is high over black). It fails (old calibration kept) if any sensor's groups are closer than `IR_CAL_MIN_CONTRAST` (250 ≈ 0.2 V; was a 600 min/max range, which failed at larger heights and silently left the 3 mm defaults in use). Result → `sharedCalibrationDone()` → telemetry task saves it to NVS (namespace `sensors`, `CAL_VERSION` 2, tied to a hash of the pins) and sends `{"type":"cal"}` with `white`, `bg[]`, `line[]`; the phone shows the line colour and the weakest sensor's line/background gap in volts (the margin left at that height). Loaded at boot in every build; defaults `IR_DEFAULT_WHITE/BLACK` (3 mm values) until calibrated. **Recalibrate after changing the array height.** Host test of the maths: compile `src/sensors.cpp` with a stub `Arduino.h` (not kept in the repo).
- Sample frame **version 2** adds the sensor count and one level byte per sensor (bytes 36+); the phone shows each sensor as a grey level.
- Make the sensor count and weights configurable so the 16-channel array and an analog version can replace it later without touching control or telemetry code.

### Control
- Position error → steering correction → left/right motor duty, clamped to `MAX_DUTY`.
- Gains (`KP`, `KD`, base speed) in `config.h`, and tunable live from the phone UI later.
- Filter the D term if added.
- **Built 2026-10-15:** PD in `controlUpdate()`. The derivative of position is low-pass filtered (`KD_FILTER_MS` = 10 ms) and reset by `controlReset()` whenever line following (re)starts (mode change, end of start delay, after a lost-line stop). `KD` defaults to 0 (P only) until tuned.
- **Sharp (90°) corners (built 2026-10-22):** when the line is Lost the robot **spins on the spot** (`PIVOT_SPEED` 0.6, wheels opposite) toward the side it went, until a sensor sees the line, then PD restarts with `controlReset()`. Brakes after `LOST_TIMEOUT_MS` (1500 ms, was 500) without the line. Before this, a lost line only meant PD at position ±1, a wide forward arc (inner wheel ~0.2 at speed 0.7, KP 0.4) that couldn't make a 90° corner. The side comes from `lastSide` in `readPosition()`: the outer sensor that last saw the line alone (the corner's side branch reaches it), else the last position beyond `LOST_SIDE_MIN` (0.2). Not just the sign of the last position, because at a corner the line can vanish under the middle sensors last. Phone status shows "Lost → turning left/right".
- **Corner brake (built 2026-10-22):** above ~50 % speed the robot slid too far past a 90° corner for the spin to swing the array back onto the side branch (KP doesn't help: it's momentum). Now both wheels go **full reverse for `cornerBrakeMs`**, once per corner, when the corner shows (`sensorsCorner()`: ≥ `CORNER_MIN_SENSORS` (4) on the line reaching exactly one outer sensor; a cross reaches both and doesn't count) or when the line is lost. Re-arms after `CORNER_REARM_MS` (200 ms) of plain line following, so it can't fire twice per corner or from a standstill; disarmed outside Auto. `cornerBrake()` in `control_task.cpp`.
- **Corner brake and spin speed are live sliders** (Tuning fields `cornerBrakeMs` 0–300 ms, default `CORNER_BRAKE_MS` 40; `pivotSpeed` 0.1–1, default `PIVOT_SPEED` 0.6). Message `0x12` is now 29 bytes (a 21-byte one keeps the last two). Tuning record `RECORD_VERSION` 2; a version-1 record (five fields) is migrated, so saved slider values survive the update. User's values on 2026-10-22: speed 0.5, KP 1.0, KD 0.005 turns 90° without the brake; 60 %+ with KP 1.5 didn't.
- **Motor balance trim** (`MOTOR_TRIM`, `motorsSetTrim()`): > 0 slows the right motor, < 0 the left, applied in every mode. Default **0** for the new 3 V motors (the old 0.28 was measured on the previous motors). Telemetry speed is after trim.

## Telemetry

### Transport
- ESP32 runs as its **own Wi-Fi access point** and serves the web page itself (no external server, no internet).
- Phone joins the ESP32 Wi-Fi and opens `192.168.4.1`. Data streams over **WebSocket**.
- Disable Wi-Fi power saving (`WiFi.setSleep(false)`).

### Frame content
Send at a steady 30–50 Hz from the telemetry task, as a compact **binary** frame:
- timestamp (ms)
- sensor bits (1 byte, one bit per sensor)
- position / error
- left and right motor duty
- current lap number
- last lap time (ms)

Send lap completion as a separate **event message** so it is never lost to dropped frames. Reserve room in the frame format for later fields (speed from encoders, current/voltage from INA219).

### Implementation (Phases 3–5, built 2026-10-15)
- `src/control_task.cpp`: control loop task, core 1, priority `configMAX_PRIORITIES - 3`, paced by `vTaskDelayUntil` (1 ms tick, so `CONTROL_PERIOD_US` must be whole ms). Reads and publishes sensors during the start delay too, motors braked.
- `src/shared.cpp`: latest-sample mailbox (`portMUX` critical section, struct copy).
- `src/telemetry.cpp`: core 0 task. Wi-Fi AP `WIFI_SSID` / `WIFI_PASS` (in `config.h`), page on port 80, WebSocket on port 81 (library `links2004/WebSockets`, polled from this task, so no AsyncTCP core flag needed). The frame layout is documented in `src/telemetry.h`: type 1 sample, 36 bytes, little-endian, version 1. Sensor bits are **u16** (not 1 byte) so the 16-channel array fits. Bytes 28–35 are reserved for encoder speed and INA219. Type 2 (lap event) is reserved for Phase 6. A JSON `hello` text message on connect carries the sensor count and gains.
- `web/index.html`: the phone page, embedded with `board_build.embed_txtfiles` (edit the file, then rebuild). Open it with `?demo` for fake data (`.claude/launch.json` → `phone-page` serves `web/` on port 8765).
- The frame also carries `loop max us` and `period max us` per 20-tick window, to prove Wi-Fi doesn't disturb the control loop.
- **ESP32 performance** (`src/sysmon.cpp`, built 2026-10-15): a `{"type":"stats"}` text message once a second with CPU load per core, control-loop load, free/lowest heap, free PSRAM, chip temperature, phone RSSI and uptime. FreeRTOS run-time stats are off in the Arduino core, so CPU load comes from idle hooks: each hook returns `false` (idle task keeps spinning, no wait-for-interrupt) and adds up gaps < 50 µs between its calls as idle time. This is an estimate: interrupts during idle count as idle.

### Phone web page
- Single self-contained `index.html` (inline JS and CSS, **no CDN** — the AP has no internet), embedded in the sketch or stored in LittleFS.
- Smooth UI rules:
  - WebSocket messages only write into a ring buffer.
  - One `requestAnimationFrame` loop draws a `<canvas>` graph at screen refresh rate.
  - Ease numeric readouts toward their targets instead of jumping.
  - Keep the DOM tiny: no frameworks.
- Show: live sensor bits, position/error graph, motor duty, running lap time, last lap, best lap, and a lap list.
- Later: a panel to change `KP`, `KD`, and base speed live.

## Drive modes and manual control (built 2026-10-15)
- Three modes chosen on the phone page: **Auto** (line following), **Manual** (on-screen joystick), **Stop** (motors braked, sensors and telemetry still live). Boot mode = `BOOT_MODE` in `config.h` (Stop). The start delay applies to every mode.
- Phone → robot messages over the same WebSocket (layout in `src/telemetry.h`): `0x10` set mode, `0x11` drive (throttle, steer). The command mailbox lives in `src/shared.cpp` (`sharedSetMode`, `sharedSetDrive`, `sharedCommand`).
- Manual mixing is in `controlManual()` (`src/control.cpp`): arcade mix, scaled together so the turn ratio is kept, capped at `MANUAL_MAX_SPEED` (1.0 = 3 V since 2026-10-22).
- **Dead-man:** the phone repeats drive commands at 30 Hz while in Manual. The robot brakes if none arrive for `MANUAL_TIMEOUT_MS` (300 ms): Wi-Fi drop, locked phone, closed tab. The phone also releases the stick when the page is hidden.
- The current mode is echoed in every sample frame (flags bits 1–3), so the page always shows what the robot is actually doing.
- **Center test** (mode `Hold` = 4, phone button "Center test", built 2026-10-22): bench test of the steering. No forward speed: `controlCorrection()` (the same PD as Auto, live KP/KD) clamped to ±1 drives the wheels opposite (`motorsSet(c, -c)`), so the robot turns on the spot to keep the line centred. The user slides or twists it off-centre and watches it come back (overshoot → lower KP / raise KD; slow or stops short → raise KP). Lost line: spins toward the remembered side like Auto, brakes after `LOST_TIMEOUT_MS`. No corner brake in this mode; no flash saves while it drives.
- **Live tuning sliders** (phone "Tuning" card, below the graph): Auto speed, Manual max speed, KP, KD, Motor balance, Corner brake, Corner spin speed. Message `0x12` carries all seven as f32; the robot clamps them in `sharedSetTuning()` (a separate `Tuning` mailbox), applies them in the control task each tick, and sends `{"type":"tuning"}` to every phone so sliders agree. The card also shows the matching `config.h` line.
- **Saved to flash** (`src/settings.cpp`, NVS namespace `tuning`, built 2026-10-15): loaded at boot in `main.cpp`. The telemetry task saves `TUNING_SAVE_DELAY_MS` (2 s) after the last slider change, skips writes that change nothing, and **never saves while Auto is driving** (a flash write pauses both cores); it waits until the robot stops. The record holds an FNV-1a hash of the `config.h` defaults, so **editing `config.h` defaults discards the saved values** at the next boot. The phone shows the save state (`"save"` in hello/tuning: saved, defaults, pending, waiting, error) and has a "Reset to config.h values" button (message `0x13`, erases the record).

## Battery estimate (built 2026-10-16)
- Battery: 2S 18650, **2600 mAh** each in series (`BATTERY_MAH`, `BATTERY_NOMINAL_V` 7.4 V, `BATTERY_USABLE` 0.9 → ~17.3 Wh usable). BMS and MINI560 are on the pack; the board only sees 5 V.
- `src/power.cpp`: own INA219 driver (no library), task on core 0 at 50 Hz, 0.1 Ω shunt, ±320 mV range, 16-sample averaging. Integrates battery energy = (Vin+ × I) / `BUCK_EFFICIENCY` (0.88). % = 1 − used / usable; time left uses a ~30 s power average.
- **Only correct if the phone's "Battery charged" button (message `0x14`) is pressed after each full charge.** Energy used is saved to NVS (namespace `battery`) about once a minute and right after a reset, never while Auto is driving (`powerService()` from the telemetry task).
- **Battery low** = MINI560 output (Vin+) below `RAIL_LOW_V` (4.75 V) for 2 s while current > 50 mA. Running on USB with the switch off draws no shunt current, so it neither counts energy nor triggers the warning.
- Frame bytes 32–35 carry 5 V rail mV and current mA live (0 if the INA219 isn't found); the 1 s `stats` message carries the battery fields. Phone: battery badge in the header plus a Battery card.

## Lap timing
- Measured **on the ESP32** with `micros()`, never on the phone.
- Detect a lap when all (or nearly all) sensors read black at the same time (start/finish bar wider than the line), confirmed over a couple of consecutive samples.
- Add a **lockout time** after each crossing (about 70% of the expected lap time) so one bar is not counted twice.
- First crossing starts the clock. Check the actual Cytron track layout for crossings or junctions that could cause false triggers; if there are any, discuss adding a side sensor with the user.
- The phone interpolates the running timer from the last lap-start event.

**Status: switched off** (`LAP_TIMING_ENABLED = false`, user decision 2026-10-15). The code below stays in place; the phone hides the lap cards when the `hello` message says `"laps":false`.

### Implementation (Phase 6, built 2026-10-15)
- **Lap marker = Cytron Cross tile** (user's decision; the competition track's marker doesn't matter). Its 18 mm perpendicular line spans the whole array.
- `src/lap_timer.cpp`, called from the control task only while running (not during the start delay). A crossing = at least `LAP_MIN_SENSORS` (all 5) sensors each saw black within the last `LAP_WINDOW_MS` (30 ms), so a robot crossing at an angle still counts. Lockout = `LAP_MIN_LOCKOUT_MS` (3 s) until the first lap, then `max(3 s, 0.7 × time between crossings)`. `LAP_CROSSINGS_PER_LAP` = 2 for a figure-8 through the cross.
- Events go control task → FreeRTOS queue (`sharedPushLap`, never blocks) → telemetry task → WebSocket type 2 message. The telemetry task keeps the history (200 events) and replays it to every phone that connects.
- Station tiles would also trigger a lap (all black), and T-junctions can pull the robot into the branch; keep them out of the practice layout.

## Versions
- **`v1success`** (git tag on branch `dev`, 2026-10-22): last version that worked on the track. Auto calibration (threshold + line colour), spin toward the remembered side when the line is lost. Turns 90° corners at speed 50 %, KP 1.0, KD 0.005. No corner brake.

## Suggested phases
1. Project skeleton: `config.h`, pin map, motor driver functions, `MAX_DUTY`, motor test.
2. Digital sensor reading + `readPosition()` + P control, auto-start on boot.
3. Dual-core split, shared struct, fixed-rate control loop.
4. Wi-Fi AP + web server + WebSocket + binary telemetry frame.
5. Smooth canvas web UI.
6. Lap timing with lockout + lap events + UI display.
7. D term, live tuning from the phone.
8. Later: encoders, INA219, analog reading, 16-channel array.

## Working style
- Build one phase at a time and confirm it works before moving on.
- Keep code modular (motors, sensors, control, telemetry, lap timer in separate files).
- Ask the user about anything marked TBD instead of guessing pins or hardware details.
- Confirm the build system with the user (Arduino IDE vs PlatformIO) before creating the project structure.

# Line Follower Robot (ESP32-S3)

A speed-race line follower built for an Industrial Design course. All robots race on the same
track at the end of the semester; the fastest wins. This repository holds everything: the
firmware, the phone app, the chassis design (3D models) and the schematic.

**Detailed technical notes:** [CLAUDE.md](CLAUDE.md) records every design decision, measurement
and fix, with dates. It is the best source for the report.

## What the robot does

- Follows a white or black line with an 8-sensor analog IR array, using PD steering.
- Calibrates itself: one spin on the line sets the threshold and works at any sensor height.
- Handles the hard parts of a track:
  - **sharp 90° corners:** brakes and spins toward the side the line went,
  - **hill crests and gaps:** drives straight on when the line vanishes from the middle,
  - **figure-8 crossings:** holds its heading over a crossing instead of turning into it.
- **Speed control:** wheel encoders measure the real speed, and the robot holds a target speed
  in mm/s uphill, downhill and on a draining battery.
- **Track mapping:** in **Learn** mode it records the track; in **Race** mode it plans its speed
  from the map, fast on straights and slowing before curves and corners.
- **Phone app:** the robot makes its own Wi-Fi network and serves a web page with live sensors,
  graphs, drive modes, a joystick, live tuning sliders, battery level and diagnostics. No
  internet or app install needed.
- **Wireless (OTA) firmware updates** and a voltage-based battery percentage.

## Hardware

| Part | Used |
|---|---|
| Microcontroller | YD ESP32-S3 N16R8 (16 MB flash, 8 MB PSRAM) |
| Line sensor | 8-channel analog IR array (QTR-8A clone) |
| Motors | 2 × N20 6 V gear motors with hall encoders |
| Motor driver | DRV8833 |
| Battery | 2S Li-ion (8.4 V full) |
| Regulator | MINI560 5 V buck converter |
| Power monitor | INA219 (battery voltage and current) |

Full parts list: [line_follower_bom.xlsx](line_follower_bom.xlsx).
Schematic: [schematic/pcb_chassis_sheet1.png](schematic/pcb_chassis_sheet1.png).
Pin map: [CLAUDE.md](CLAUDE.md#pin-map-source-of-truth-the-schematic-all-pins-in-includeconfigh)
and [include/config.h](include/config.h).

## Repository layout

| Folder / file | Contents |
|---|---|
| [`src/`](src) | Firmware source (C++, Arduino framework on PlatformIO), one file per part: see below |
| [`include/config.h`](include/config.h) | All pins and settings, each explained |
| [`web/index.html`](web/index.html) | The phone app (served by the robot itself) |
| [`3D Model/`](3D%20Model) | Chassis, body and part models (STL for printing, STEP, Fusion 360 `.f3d`) |
| [`3D Model/chassis/make_chassis.py`](3D%20Model/chassis/make_chassis.py) | Fusion 360 script that generates the chassis, with all dimensions |
| [`schematic/`](schematic) | Circuit schematic of the PCB chassis |
| [`rpmtest/`](rpmtest) | Small motor speed test program |
| [`platformio.ini`](platformio.ini) | Build settings and test programs |
| [`CLAUDE.md`](CLAUDE.md) | Full engineering notes and decision log |

### Firmware files (`src/`)

| File | Role |
|---|---|
| `main.cpp` | Start-up: loads saved settings, starts the tasks |
| `sensors.cpp` | IR array reading, calibration, line position, corner / crossing / gap detection |
| `control.cpp` | Steering (PD) and speed control |
| `control_task.cpp` | The 1 kHz control loop (CPU core 1): drive modes, corners, crests |
| `motors.cpp` | Motor driver and voltage limit (from the measured battery voltage) |
| `encoders.cpp` | Wheel encoders (hardware pulse counters), distance and speed |
| `track_map.cpp` | Track mapping: Learn and Race |
| `telemetry.cpp` | Wi-Fi, web server, live data to the phone (CPU core 0) |
| `power.cpp` | Battery voltage, current and percentage (INA219) |
| `settings.cpp` | Saving settings, calibration and the track map in flash |
| `ota.cpp` | Wireless firmware updates |
| `sysmon.cpp` | CPU load, memory and temperature for the phone |
| `lap_timer.cpp` | Lap timing (built, currently switched off) |
| `tests.cpp` | Bench tests: motors, sensors, steering, encoders |

## Versions

| Tag | What it is |
|---|---|
| [`v1success`](../../tree/v1success) | First version that worked on the track: auto calibration, 90° corners at moderate speed |
| [`v2success`](../../tree/v2success) | PCB chassis support, OTA updates, battery %, hill and crossing handling |
| [`v3`](../../tree/v3) | Encoder motors (6 V), speed control |
| `main` (latest) | Track mapping (Learn / Race), corner and encoder fixes, phone diagnostics |

## Building and uploading the firmware

1. Install [PlatformIO](https://platformio.org/) (VS Code extension or command line).
2. Copy `secrets.example.ini` to `secrets.ini` and set the robot's Wi-Fi and upload passwords.
   `secrets.ini` is never committed.
3. Connect the robot by USB and run:
   ```bash
   pio run -e robot -t upload
   ```
   Later uploads can go over Wi-Fi: join the robot's network, then `pio run -e robot_ota -t upload`.
4. On a phone, join the robot's Wi-Fi network and open `http://192.168.4.1`.

Bench tests: `pio run -e motortest`, `-e sensortest`, `-e steertest` or `-e enctest`, each with
`-t upload -t monitor`.

#pragma once
// Network task pinned to core 0: Wi-Fi access point, the phone page on
// port 80, and a WebSocket on WS_PORT streaming binary frames.
//
// Messages sent to the phone:
//   text   {"type":"hello",...}   once per connection (sensor count, rated motor
//          volts "motorV", voltage limit "vLimit" / max "vMax", "inaBattery"
//          (the INA219 measures the battery, not the 5 V rail), calibration
//          "saved"/"default", line colour "white", tuning)
//   text   {"type":"stats",...}   once a second: CPU load per core, control loop
//          load, heap, PSRAM, chip temperature, phone RSSI, uptime (see sysmon.h),
//          and the battery estimate (see power.h)
//   text   {"type":"cal","ok":..,"bad":..,"saved":..,"white":..,"bg":[..],"line":[..]}
//          after each calibration run (bad = first sensor that failed, or -1)
//   binary type 1 sample frame, TELEMETRY_HZ, 37 + IR_COUNT bytes, little-endian:
//     0  u8  type = 1            1  u8  version = 2
//     2  u16 sensor bits         4  u32 time ms
//     8  i16 position x10000    10  i16 left speed x10000 (1.0 = rated motor V)
//    12  i16 right speed x10000 14  u8  line state (0 on, 1 gap: gone from the
//                                     middle, driving straight; 2 lost)
//    15  u8  flags (bit0 running, bits1-3 drive mode: 0 auto, 1 manual, 2 stop,
//              3 calibrate, 4 center test; bit4 crossing: position held)
//    16  u16 lap number         18  u16 reserved
//    20  u32 last lap ms
//    24  u16 loop max us        26  u16 period max us
//    28  i16 left speed  (reserved: encoders)
//    30  i16 right speed (reserved: encoders)
//    32  u16 INA219 bus mV: 5 V rail, or the battery on PcbChassis (0 if not found)
//    34  i16 current mA  (INA219, whole robot)
//    36  u8  sensor count n     37  u8 x n sensor levels (0 = background, 255 = line)
//   binary type 2 lap event, 16 bytes, little-endian:
//     0  u8  type = 2            1  u8  version = 1
//     2  u16 laps completed (0 = timing just started)
//     4  u32 lap time ms (0 for the start event)
//     8  u32 current lap start, robot ms (same clock as the sample time)
//    12  u8  flags (bit0 replay: history re-sent on connect)
//    13..15 reserved
//   On connect the robot replays every lap event so far, so a phone that
//   joins or reconnects mid-race gets the full lap list and running timer.
//
// Messages from the phone (binary, little-endian):
//   type 0x10 set mode, 2 bytes:  0 u8 type, 1 u8 mode (0 auto, 1 manual, 2 stop,
//                                    3 calibrate: spin, save, then stop,
//                                    4 center test: turn on the spot only)
//   type 0x11 manual drive, 5 bytes: 0 u8 type, 1 i16 throttle x10000,
//                                    3 i16 steer x10000
//   type 0x12 tuning, 49 bytes: 0 u8 type, then f32 each: 1 auto speed,
//                                  5 manual max, 9 kp, 13 kd, 17 motor trim,
//                                  21 corner brake ms, 25 spin speed,
//                                  29 calibrate spin speed, 33 smoothing ms,
//                                  37 hill/gap straight ms, 41 downhill speed
//                                  factor, 45 downhill ms
//                                  (the robot clamps every field; a shorter
//                                  older message keeps the fields it lacks)
//   type 0x13 reset tuning, 1 byte: erase the saved tuning, back to config.h
//   type 0x14 battery charged, 1 byte: battery estimate back to 100 %
//   type 0x16 motor voltage limit (dev unlock), 5 bytes: 0 u8 type, 1 f32 volts
//                                   at speed 1.0, clamped to rated .. MOTOR_UNLOCK_MAX_V;
//                                   not saved; sent back as {"type":"vlimit","v":..}
//   type 0x15 line colour, 2 bytes: 0 u8 type, 1 u8 (1 white line, 0 black);
//                                   saved to flash, sent back as
//                                   {"type":"line","white":..} to every phone
//   In manual mode the phone repeats 0x11 ~30 times/s; the robot brakes if
//   none arrive for MANUAL_TIMEOUT_MS.
//   After 0x12/0x13 the robot sends {"type":"tuning",...} to every phone; the
//   same fields are in hello. "save" is the flash state: saved, defaults
//   (nothing saved, config.h values), pending, waiting (Auto is driving),
//   error.

void telemetryStart();

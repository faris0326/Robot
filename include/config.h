#pragma once
// All pins and tunable constants live here. Motor and I2C pins come from the
// EasyEDA schematic; the 8-channel IR array pins from the user (2026-10-21).

#include <stdint.h>

// ---------------------------------------------------------------------------
// Motor driver (DRV8833, powered from the +5 V rail)
// ---------------------------------------------------------------------------
// Right motor: IN1/IN2 -> OUT1/OUT2 -> M1_R/M2_R
constexpr uint8_t PIN_MOTOR_R_IN1 = 18;
constexpr uint8_t PIN_MOTOR_R_IN2 = 17;
// Left motor:  IN3/IN4 -> OUT3/OUT4 -> M1_L/M2_L
constexpr uint8_t PIN_MOTOR_L_IN1 = 16;  // DRV8833 IN3
constexpr uint8_t PIN_MOTOR_L_IN2 = 15;  // DRV8833 IN4

// Flip these if a wheel spins backwards on a "forward" command.
constexpr bool MOTOR_L_INVERT = false;  // new 3 V motors: confirmed with motortest 2026-10-22
constexpr bool MOTOR_R_INVERT = false;

// Slow decay (drive/brake) gives a more linear speed vs duty curve than
// fast decay (drive/coast). Keep true unless there's a reason not to.
constexpr bool MOTOR_SLOW_DECAY = true;

// Motor balance. > 0 slows the right motor by that fraction, < 0 slows the
// left. Reset to 0 for the new 3 V motors (2026-10-21); the old 0.28 was
// measured on the previous motors. Check on the floor: in Manual, push the
// stick straight up; the robot should drive straight. Adjust live with the
// phone's "Motor balance" slider.
constexpr float MOTOR_TRIM = 0.0f;

constexpr uint32_t MOTOR_PWM_FREQ_HZ = 20000;  // above hearing range
constexpr uint8_t  MOTOR_PWM_BITS    = 10;     // 0..1023

// Motor voltage cap. The motors are rated 3 V but the driver runs from the
// 5 V rail, so PWM duty never exceeds 3 / 5 = 60 %. Every speed in the code
// and on the phone is a fraction of the rated voltage: 1.0 (100 %) = 3 V.
constexpr float MOTOR_RATED_V  = 3.0f;
constexpr float MOTOR_SUPPLY_V = 5.0f;
constexpr float MAX_DUTY = MOTOR_RATED_V / MOTOR_SUPPLY_V;  // PWM duty at speed 1.0
static_assert(MAX_DUTY > 0.0f && MAX_DUTY <= 1.0f, "rated voltage must not exceed the supply");

// ---------------------------------------------------------------------------
// Startup
// ---------------------------------------------------------------------------
// No start button: wait this long after boot before the motors may move.
constexpr uint32_t START_DELAY_MS = 3000;

// ---------------------------------------------------------------------------
// IR array: 8-channel analog QTR-8A clone, powered from 3.3 V (its 3.3V pads
// are bridged, so never 5 V). Each D pin is an analog voltage: ~0.3 V over
// white, ~2.8 V over black at ~3 mm.
// ---------------------------------------------------------------------------
constexpr uint8_t IR_COUNT = 8;
// Listed physically left -> right, looking from behind the robot. D1 is on
// the RIGHT (confirmed with sensortest 2026-10-22), so the list runs D8..D1.
// Sensor pins must be ADC1 (GPIO 1..10, not strapping pin 3); ADC2 pins
// (GPIO 11..20) can't be read while Wi-Fi is on. The build checks this.
// Wiring: D1=1, D2=2, D3=4, D4=5, D5=6, D6=7, D7=8, D8=9.
constexpr uint8_t PIN_IR[IR_COUNT] = {9, 8, 7, 6, 5, 4, 2, 1};  // D8..D1, left -> right
constexpr uint8_t PIN_IR_LED = 11;  // the board's IR pin: HIGH = emitters on

// Line colour until the robot is calibrated (calibration measures it and
// replaces this). The race track (Cytron) is a black line on white; true =
// a white line on a dark surface.
constexpr bool LINE_IS_WHITE = true;

// Position weight per sensor, left -> right (evenly spaced). Only the ratios
// matter: readPosition() scales them so the outermost sensor gives -1 / +1.
constexpr float IR_WEIGHTS[IR_COUNT] = {-7, -5, -3, -1, 1, 3, 5, 7};

// Each sensor's reading is calibrated to a level: 0 = background, 1 = line.
// 0.5 = halfway between the background and line readings calibration measured.
constexpr float IR_ON_LINE     = 0.5f;   // level that counts as "on the line"
constexpr float IR_NOISE_FLOOR = 0.05f;  // levels below this are ignored in the position

// Raw ADC readings (12-bit) used until the robot has been calibrated,
// measured with the array ~3 mm above the surface.
constexpr uint16_t IR_DEFAULT_WHITE = 400;   // ~0.3 V
constexpr uint16_t IR_DEFAULT_BLACK = 3600;  // ~2.8 V
// Calibration fails if any sensor's typical line and background readings are
// closer than this (it never crossed the line, it's disconnected, or the
// array is too high to tell them apart). 250 = ~0.2 V; ADC noise is ~30.
constexpr uint16_t IR_CAL_MIN_CONTRAST = 250;

// Calibration run (phone "Calibrate" button): with the robot on the line, it
// spins right, left, right on the spot while recording each sensor's min/max.
constexpr float    CAL_SPIN_SPEED  = 0.5f;   // fraction of rated voltage
constexpr uint32_t CAL_DURATION_MS = 2400;   // 1/4 right, 1/2 left, 1/4 right

// ---------------------------------------------------------------------------
// Control (position is -1 = line far left .. +1 = line far right)
// ---------------------------------------------------------------------------
// Control loop period. It is paced by the FreeRTOS tick (1 ms), so this
// must be a whole number of milliseconds.
constexpr uint32_t CONTROL_PERIOD_US = 1000;  // 1 kHz

// Speeds are fractions of the motors' rated voltage (1.0 = 3 V).
constexpr float BASE_SPEED = 1.00f;  // speed when the line is centred
constexpr float KP         = 0.40f;  // speed difference per unit of position
// D term: speed per (position unit per second). 0 = P only. Tune live from
// the phone: raise KP until the robot weaves, then raise KD until it settles.
constexpr float KD         = 0.000f;
// The derivative of position is low-pass filtered with this time constant.
constexpr float KD_FILTER_MS = 10.0f;

// All of the above (plus MOTOR_TRIM and the manual speed) can be changed
// live from the phone; these are the values after every power-on.

// Sharp (90-degree) corners: when no sensor sees the line, the robot spins on
// the spot toward the side it went, wheels in opposite directions.
constexpr float PIVOT_SPEED   = 0.60f;  // fraction of rated voltage
// A position beyond this (either side) marks which way the line went, when
// no outer sensor saw it alone.
constexpr float LOST_SIDE_MIN = 0.20f;

// No sensor sees the line for longer than this -> brake until it is seen
// again (long enough to spin past 90 degrees). GAP_TIMEOUT_MS only applies to
// digital arrays, where the line can sit between two sensors; the analog
// array never reports a gap.
constexpr uint32_t LOST_TIMEOUT_MS = 1500;
constexpr uint32_t GAP_TIMEOUT_MS  = 2000;

// ---------------------------------------------------------------------------
// Drive modes, switched from the phone page
// ---------------------------------------------------------------------------
// Calibrate runs the sensor calibration spin, then switches to Stop.
enum class DriveMode : uint8_t { Auto = 0, Manual = 1, Stop = 2, Calibrate = 3 };

constexpr DriveMode BOOT_MODE = DriveMode::Auto;  // mode after power-on

// Manual: phone joystick. Throttle and steer are mixed into left/right
// speed, then scaled so full stick gives at most MANUAL_MAX_SPEED.
constexpr float    MANUAL_MAX_SPEED  = 1.00f;  // fraction of rated voltage (1.0 = 3 V)
// The phone sends ~30 commands/s while in Manual. If none arrive for this
// long (Wi-Fi drop, phone locked), the motors brake.
constexpr uint32_t MANUAL_TIMEOUT_MS = 300;

// ---------------------------------------------------------------------------
// Lap timing (switched off for now; the code stays for later)
// ---------------------------------------------------------------------------
constexpr bool LAP_TIMING_ENABLED = false;

// The lap marker is a Cytron Cross tile. Its perpendicular line covers the
// whole array, while the 18 mm line being followed covers 2-3 of 8 sensors.
constexpr uint8_t  LAP_MIN_SENSORS       = 6;  // sensors that must see the cross line
// A sensor counts if it saw black within this window. The robot rarely
// crosses dead square, so the outer sensors reach the line at different moments.
constexpr uint32_t LAP_WINDOW_MS         = 30;
constexpr uint8_t  LAP_CROSSINGS_PER_LAP = 1;         // 2 for a figure-8 through the cross
constexpr uint32_t LAP_MIN_LOCKOUT_MS    = 3000;      // ignore the cross this long after a crossing
constexpr float    LAP_LOCKOUT_FRACTION  = 0.7f;      // then: this x the time between crossings

// ---------------------------------------------------------------------------
// Telemetry: the robot is its own Wi-Fi access point. Join it from the phone
// and open http://192.168.4.1
// ---------------------------------------------------------------------------
constexpr char     WIFI_SSID[]      = "Faris's_ESP";
constexpr char     WIFI_PASS[]      = "12345678";  // 8..63 characters, or "" for an open network
constexpr uint8_t  WIFI_CHANNEL     = 6;
constexpr uint8_t  WIFI_MAX_CLIENTS = 2;
constexpr uint16_t WS_PORT          = 81;  // WebSocket; the page is on port 80
constexpr uint32_t TELEMETRY_HZ     = 40;  // frames per second sent to the phone
// Tuning from the phone is saved to flash this long after the last change,
// and never while Auto is driving (a flash write pauses both cores briefly).
constexpr uint32_t TUNING_SAVE_DELAY_MS = 2000;

// ---------------------------------------------------------------------------
// Power monitor: INA219 in series with the 5 V rail on the board. The BMS
// and MINI560 sit on the battery pack, so the battery itself can't be
// measured; its charge is estimated by counting the energy the robot uses.
// ---------------------------------------------------------------------------
constexpr uint8_t PIN_I2C_SDA = 41;
constexpr uint8_t PIN_I2C_SCL = 42;
constexpr uint8_t INA219_ADDR = 0x40;            // A0/A1 open on the module
constexpr float   INA219_SHUNT_OHMS = 0.1f;      // "R100" resistor on the module

constexpr float BATTERY_MAH       = 2600.0f;     // 2S: two 2600 mAh 18650s in series
constexpr float BATTERY_NOMINAL_V = 7.4f;        // 2 x 3.7 V
constexpr float BATTERY_USABLE    = 0.90f;       // share a real pack delivers before the BMS cuts off
constexpr float BUCK_EFFICIENCY   = 0.88f;       // MINI560 near 5 V out
// The MINI560 can't hold 5 V once the battery is nearly empty. Its output
// below this for 2 s means "battery low".
constexpr float RAIL_LOW_V        = 4.75f;

// Encoders: not used. The new 3 V motors (2026-10-21) are run without them;
// GPIO 11 is now the IR LED pin.

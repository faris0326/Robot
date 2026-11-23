#pragma once
// All pins and tunable constants live here. Motor and I2C pins come from the
// EasyEDA schematic; the 8-channel IR array pins from the user (2026-10-21).

#include <stdint.h>

// ---------------------------------------------------------------------------
// Board
// ---------------------------------------------------------------------------
//   Original    the first build (schematic 2026-10-16): INA219 on the 5 V
//               rail, IR LED enable on GPIO 11. Its DRV8833 was rewired to
//               the 2S battery on 2026-10-23, where nothing measures it, so
//               the motor supply is assumed to be MOTOR_SUPPLY_V.
//   PcbChassis  the PCB-as-chassis board (schematic/pcb_chassis_sheet1.png):
//               INA219 on the battery side (pack voltage and all current,
//               motors included), DRV8833 fed after it, IR LED enable on
//               GPIO 10, encoder connectors on GPIO 11..14. The motor supply
//               is measured, so the motors get their voltage limit at any
//               charge.
// Everything else (sensor, motor and I2C pins) is the same on both.
enum class Board : uint8_t { Original, PcbChassis };
constexpr Board BOARD = Board::PcbChassis;
constexpr bool  INA219_ON_BATTERY = BOARD == Board::PcbChassis;

// ---------------------------------------------------------------------------
// Motor driver (DRV8833, powered from the 2S battery)
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

// Motor voltage cap. The motors are rated 3 V; the driver (DRV8833 VM) runs
// straight from the 2S battery (the MINI560 only feeds the ESP32 and sensors).
// PWM duty = limit / supply voltage. Every speed in the code and on the phone
// is a fraction of the limit: 1.0 (100 %) = 3 V unless unlocked (below).
//
// MOTOR_SUPPLY_V is the supply assumed when it isn't measured (Original
// board, or the INA219 not answering): a full battery, so the motors never
// see more than the limit; as the battery runs down they get less (7.4 V:
// 2.6 V). PcbChassis measures it instead (MOTOR_SUPPLY_* below).
constexpr float MOTOR_RATED_V  = 3.0f;
constexpr float MOTOR_SUPPLY_V = 8.4f;  // 2S full charge (was 5.0 from the MINI560)
constexpr float MAX_DUTY = MOTOR_RATED_V / MOTOR_SUPPLY_V;  // PWM duty at speed 1.0
static_assert(MAX_DUTY > 0.0f && MAX_DUTY <= 1.0f, "rated voltage must not exceed the supply");
// Dev unlock (phone "Dev" card): the motor voltage limit can be raised up to
// this for more speed, overdriving the 3 V motors (more heat and wear).
// Speed 1.0 then means the unlocked voltage, so every speed scales with it.
// It is not saved: every power-on starts at MOTOR_RATED_V.
constexpr float MOTOR_UNLOCK_MAX_V = 5.0f;
static_assert(MOTOR_UNLOCK_MAX_V >= MOTOR_RATED_V && MOTOR_UNLOCK_MAX_V <= MOTOR_SUPPLY_V,
              "unlock limit must be between the rated voltage and the supply");
// PcbChassis: the measured supply (INA219 bus voltage = the DRV8833's VM)
// replaces MOTOR_SUPPLY_V. A rise is followed at once and a drop over
// MOTOR_SUPPLY_FILTER_MS, so the duty errs low (slower), not high. A reading
// outside MIN..MAX (a fault, or a 5 V rail on the Original board) falls back
// to MOTOR_SUPPLY_V.
constexpr float MOTOR_SUPPLY_FILTER_MS = 200.0f;
constexpr float MOTOR_SUPPLY_MIN_V     = 5.5f;   // a 2S pack this low is flat (or it's a 5 V rail)
constexpr float MOTOR_SUPPLY_MAX_V     = 9.0f;   // above a full 2S pack
static_assert(MOTOR_SUPPLY_MIN_V > MOTOR_UNLOCK_MAX_V, "duty must stay below 100 % at the lowest supply");

// ---------------------------------------------------------------------------
// Startup
// ---------------------------------------------------------------------------
// The robot boots in Stop (BOOT_MODE): pick Auto on the phone to start.
// Whatever the mode, the motors can't move until this long after boot.
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
// The array's IR pin: HIGH = emitters on (its R6 pull-up keeps them on if
// the pin isn't driven).
constexpr uint8_t PIN_IR_LED = BOARD == Board::PcbChassis ? 10 : 11;

// Encoder connectors (PcbChassis only; the current N20 motors have none).
// Not used by the firmware and never driven, so they stay plain inputs. The
// encoders are powered from 5 V: check their outputs stay at 3.3 V first.
constexpr uint8_t PIN_ENC_L_C1 = 13;
constexpr uint8_t PIN_ENC_L_C2 = 14;
constexpr uint8_t PIN_ENC_R_C1 = 12;
constexpr uint8_t PIN_ENC_R_C2 = 11;

// Line colour until one is chosen on the phone (saved after that). The race
// track (Cytron) is a black line on white (false); true = a white line on a
// dark surface. Calibration measures two readings per sensor; the colour
// says which one is the line.
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
constexpr float    CAL_SPIN_SPEED  = 1.0f;   // fraction of rated voltage (1.0 = full, 3 V); live on the phone
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
// The position itself is smoothed (low-pass, this time constant) before KP
// and KD see it. Sensor (ADC) noise would otherwise reach the motors every
// 1 ms tick and make them jitter on a straight line; KD magnifies it most.
// 0 = off. Live on the phone ("Smoothing").
constexpr float POSITION_SMOOTH_MS = 4.0f;

// All of the above (plus MOTOR_TRIM and the manual speed) can be changed
// live from the phone; these are the values after every power-on.

// Sharp (90-degree) corners: when no sensor sees the line, the robot spins on
// the spot toward the side it went, wheels in opposite directions.
constexpr float PIVOT_SPEED   = 0.60f;  // fraction of rated voltage
// Corner brake: both wheels in full reverse for this long, once per corner,
// when the corner shows or the line is lost, so the robot doesn't slide past
// the corner before it can spin back to it. 0 = off. Live on the phone.
constexpr float    CORNER_BRAKE_MS    = 40.0f;
// A corner shows as a wide line: at least this many sensors on the line,
// reaching one outer sensor (a followed line covers 2-3 sensors).
constexpr uint8_t  CORNER_MIN_SENSORS = 4;
// Crossing (figure-8 track): when the array sees two separate patches of
// line, or a line this wide (sensors), the robot holds the position from
// just before and drives over instead of turning into the other line. A 90°
// corner looks wide too: held straight, its line then runs out to that side
// and the lost-line logic turns into it.
constexpr uint8_t  CROSSING_MIN_SENSORS = 4;
// The brake re-arms after this long following a plain line, so it can't fire
// again in the same corner, nor from a standstill right after a start.
constexpr uint32_t CORNER_REARM_MS    = 200;
// A position beyond this (either side) marks which way the line went, when
// no outer sensor saw it alone.
constexpr float LOST_SIDE_MIN = 0.20f;

// Line gone from the MIDDLE of the array (no outer sensor saw it within
// GAP_EDGE_RECENT_MS and the position was within LOST_SIDE_MIN): a hill
// crest, where the sensor boom sticks out over the top, or a gap in the line.
// The robot then drives straight on (no corner brake) for gapMs (live slider
// "Hill / gap", default GAP_STRAIGHT_MS) before searching like a lost line.
constexpr float    GAP_STRAIGHT_MS    = 300.0f;
// Downhill after a crest: gravity speeds the robot up beyond what the motor
// voltage gives on the flat (no encoders to see it), so it reaches the next
// corner too fast. When the line comes back after a crest/gap that lasted
// at least DOWNHILL_MIN_GAP_MS (shorter = tape flicker, ignored), Auto drives
// at DOWNHILL_SCALE x Auto speed for DOWNHILL_MS (lower voltage also lets the
// motors hold it back). Not during the gap itself: the wheels are still on
// the way up there and need full power to get over the top. Live sliders
// "Downhill"; scale 1.0 = off.
constexpr float    DOWNHILL_SCALE      = 0.60f;
constexpr float    DOWNHILL_MS         = 1000.0f;
constexpr uint32_t DOWNHILL_MIN_GAP_MS = 40;
constexpr uint32_t GAP_EDGE_RECENT_MS = 150;

// No sensor sees the line for longer than this (plus gapMs after a gap) ->
// brake until it is seen again (long enough to spin past 90 degrees).
constexpr uint32_t LOST_TIMEOUT_MS = 1500;

// ---------------------------------------------------------------------------
// Drive modes, switched from the phone page
// ---------------------------------------------------------------------------
// Calibrate runs the sensor calibration spin, then switches to Stop.
// Hold = the phone's "Center test": no forward speed, only the PD steering
// (live KP/KD), so the robot turns on the spot to keep the line in the middle.
enum class DriveMode : uint8_t { Auto = 0, Manual = 1, Stop = 2, Calibrate = 3, Hold = 4 };

constexpr DriveMode BOOT_MODE = DriveMode::Stop;  // mode after power-on: don't move until told

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
// Over-the-air uploads (ota.h). The port must match upload_flags in
// platformio.ini [env:robot_ota]; the password lives there too ([ota]).
constexpr char     OTA_HOSTNAME[]   = "line-follower";
constexpr uint16_t OTA_PORT         = 3232;
constexpr uint32_t TELEMETRY_HZ     = 40;  // frames per second sent to the phone
// Tuning from the phone is saved to flash this long after the last change,
// and never while Auto is driving (a flash write pauses both cores briefly).
constexpr uint32_t TUNING_SAVE_DELAY_MS = 2000;

// ---------------------------------------------------------------------------
// Power monitor: INA219 (shunt Vin+ -> Vin-) in series with
//   Original:   the 5 V rail. The BMS and MINI560 sit on the battery pack, so
//               the battery itself can't be measured, and since 2026-10-23
//               the motor current bypasses the shunt.
//   PcbChassis: the battery, after the slide switch. Its bus voltage is the
//               pack voltage and its current is everything, motors included.
// The charge is estimated by counting the energy the robot uses.
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
constexpr float RAIL_LOW_V        = 4.75f;       // Original
// PcbChassis: the pack itself below this for 2 s means "battery low"
// (3.2 V per cell, under load).
constexpr float BATTERY_LOW_V     = 6.4f;
// PcbChassis: the charge % comes from the pack voltage (a Li-ion resting-
// voltage table in power.cpp), not from counting energy. While the robot
// draws current the pack voltage drops by I x its internal resistance
// (cells, BMS, switch, wiring; the shunt is outside the measurement), so
// that is added back, and the result is smoothed over BATTERY_FILTER_S.
// Below BATTERY_PRESENT_V there is no battery (e.g. USB power only).
constexpr uint8_t BATTERY_CELLS         = 2;
constexpr float   BATTERY_INTERNAL_OHMS = 0.15f;
constexpr float   BATTERY_FILTER_S      = 20.0f;
constexpr float   BATTERY_PRESENT_V     = 5.0f;

// Encoders: not used. The new 3 V motors (2026-10-21) are run without them;
// GPIO 11 is now the IR LED pin.

#pragma once
// Data passed between the control loop (core 1) and the telemetry task
// (core 0). The control loop never waits on any of these.
//   Samples:  control -> telemetry, latest-value mailbox.
//   Lap events: control -> telemetry, small queue so none are lost.
//   Commands: telemetry (phone) -> control, latest-value mailbox.
//   Tuning:   telemetry (phone) -> control, latest-value mailbox.

#include <stdint.h>

#include "config.h"
#include "lap_timer.h"

struct DriveCommand {
    DriveMode mode;
    float     throttle;    // manual: -1 (reverse) .. +1 (forward)
    float     steer;       // manual: -1 (left) .. +1 (right)
    uint32_t  driveMs;     // millis() when the last manual drive command arrived
};

// Live-tunable settings. Start from config.h at every power-on.
struct Tuning {
    float autoSpeed;  // Auto base speed, 0..1 of rated voltage   (BASE_SPEED)
    float manualMax;  // Manual full-stick speed, 0..1            (MANUAL_MAX_SPEED)
    float kp;         //                                          (KP)
    float kd;         //                                          (KD)
    float trim;       // motor balance, > 0 slows the right motor (MOTOR_TRIM)
    float cornerBrakeMs;  // reverse both wheels this long per corner (CORNER_BRAKE_MS)
    float pivotSpeed;     // spin speed when the line is lost        (PIVOT_SPEED)
    float calSpeed;       // calibration spin speed                  (CAL_SPIN_SPEED)
    float smoothMs;       // position smoothing, 0 = off             (POSITION_SMOOTH_MS)
    float gapMs;          // drive straight after a hill crest / gap (GAP_STRAIGHT_MS)
    float downhillScale;  // Auto speed factor after a crest, 1 = off (DOWNHILL_SCALE)
    float downhillMs;     // how long after the crest                 (DOWNHILL_MS)
};

struct TelemetrySample {
    uint32_t seq;          // increments every publish
    uint32_t timeMs;
    uint32_t sensorBits;   // bit i = sensor i on the line, bit 0 = leftmost
    float    position;     // -1 (left) .. +1 (right)
    float    dutyLeft;     // -1 .. +1
    float    dutyRight;
    uint8_t  lineState;    // LineState: 0 on line, 1 gap, 2 lost
    bool     crossing;     // position held over a crossing
    bool     running;      // false during the start delay
    DriveMode mode;
    uint16_t lap;          // laps completed
    uint32_t lastLapMs;
    uint16_t loopMaxUs;    // longest control-loop body in the last window
    uint16_t periodMaxUs;  // longest gap between control ticks in the last window
    uint8_t  levels[IR_COUNT];  // calibrated sensor levels, 0 = background .. 255 = line
};

void sharedBegin();  // call once before starting the tasks

void            sharedPublish(const TelemetrySample& s);
TelemetrySample sharedLatest();

bool sharedPushLap(const LapEvent& ev);  // false if the queue was full
bool sharedPopLap(LapEvent& ev);         // false if empty

void         sharedSetMode(DriveMode mode);
void         sharedSetDrive(float throttle, float steer);  // also refreshes driveMs
DriveCommand sharedCommand();

void   sharedSetTuning(const Tuning& t);  // clamps every field to a safe range
Tuning sharedTuning();

// Calibration result, control -> telemetry. Returns a counter that goes up
// with every finished calibration (0 = none yet).
void     sharedCalibrationDone(bool ok, int8_t badSensor);
uint32_t sharedCalibrationResult(bool& ok, int8_t& badSensor);

#pragma once
// Line sensor array (8-channel analog QTR-8A clone). All position logic
// lives here so the array can change without touching control or telemetry.
//
// Each sensor's raw ADC value is mapped through its calibration (the typical
// background and line readings) to a level 0..1, where 1 = fully on the line.
// The line reads higher (black line) or lower (white line) than the
// background; the line colour setting says which.
// The position is the level-weighted average of the sensor weights, so it
// moves smoothly even while the line is between two sensors.

#include <stdint.h>

#include "config.h"

enum class LineState : uint8_t {
    OnLine,  // at least one sensor sees the line
    InGap,   // digital arrays only: line between two sensors (never set here)
    Lost,    // no sensor sees the line
};

struct SensorCal {
    uint16_t background[IR_COUNT];  // typical raw ADC over the background
    uint16_t line[IR_COUNT];        // typical raw ADC over the line
};

// True when the line reads lower than the background (white line, dark floor).
inline bool calLineIsWhite(const SensorCal& c) { return c.line[0] < c.background[0]; }

void sensorsBegin();

// Reads the array and returns the line position: -1 = far left, 0 = centre,
// +1 = far right. When no sensor sees the line it returns -1 or +1 toward
// the side the line was last clearly seen on: the outer sensor that last saw
// it alone, else the last position beyond LOST_SIDE_MIN (0 if neither).
float readPosition();

// From the last read; bit i set = sensor i level above IR_ON_LINE, bit 0 = leftmost.
uint32_t  sensorsBits();
// -1 / +1 when a sharp corner to that side is under the array (a wide line,
// at least CORNER_MIN_SENSORS, reaching one outer sensor), else 0.
int8_t    sensorsCorner();
LineState lineState();
void      sensorsLevels(uint8_t* out);        // IR_COUNT levels, 0..255
void      sensorsRaw(uint16_t* out);          // IR_COUNT raw ADC values

// Calibration. Begin, then every readPosition() adds each sensor's reading
// to a histogram until End. End splits each histogram into two groups
// (darker and lighter), takes the one matching the line colour as the line and
// sets the on-line threshold halfway between them. It applies the result
// only if every sensor's groups are at least IR_CAL_MIN_CONTRAST apart; on
// failure badSensor = the first one that wasn't.
// sensorsSetCalibration() swaps a sensor's two readings if they are the
// wrong way round for the line colour (e.g. saved while the colour differed).
SensorCal sensorsCalibration();
void      sensorsSetCalibration(const SensorCal& c);
void      sensorsCalibrateBegin();
bool      sensorsCalibrateEnd(int8_t& badSensor);
void      sensorsCalibrateAbort();

// Line colour: true = white line on a dark surface, false = black line on
// white. Starts as LINE_IS_WHITE. Safe to call from any core; the next
// readPosition() applies it, swapping each sensor's calibrated readings.
void sensorsSetLineWhite(bool white);
bool sensorsLineWhite();

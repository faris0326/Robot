#pragma once
// Line sensor array (8-channel analog QTR-8A clone). All position logic
// lives here so the array can change without touching control or telemetry.
//
// Each sensor's raw ADC value is mapped through its calibration (min = the
// background, max = the line) to a level 0..1, where 1 = fully on the line.
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
    uint16_t min[IR_COUNT];  // raw ADC over the background
    uint16_t max[IR_COUNT];  // raw ADC over the line
};

void sensorsBegin();

// Reads the array and returns the line position: -1 = far left, 0 = centre,
// +1 = far right. When no sensor sees the line it returns -1 or +1 toward
// the side the line was last seen on (0 if it was last seen dead centre).
float readPosition();

// From the last read; bit i set = sensor i level above IR_ON_LINE, bit 0 = leftmost.
uint32_t  sensorsBits();
LineState lineState();
void      sensorsLevels(uint8_t* out);        // IR_COUNT levels, 0..255
void      sensorsRaw(uint16_t* out);          // IR_COUNT raw ADC values

// Calibration. Begin, then every readPosition() widens each sensor's min/max
// until End, which applies it only if every sensor saw at least
// IR_CAL_MIN_RANGE. On failure badSensor = the first sensor that didn't.
SensorCal sensorsCalibration();
void      sensorsSetCalibration(const SensorCal& c);
void      sensorsCalibrateBegin();
bool      sensorsCalibrateEnd(int8_t& badSensor);
void      sensorsCalibrateAbort();

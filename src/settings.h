#pragma once
// Tuning saved in flash (NVS), so slider changes survive power-off.
//
// The saved record carries a fingerprint of the config.h defaults. If
// config.h is edited (and reflashed), the fingerprint no longer matches and
// the saved values are ignored, so config.h edits always take effect.
//
// Flash writes pause both cores for a moment, so call settingsSave() only
// from the telemetry task and not while Auto is driving the motors.

#include "sensors.h"
#include "shared.h"

Tuning settingsDefaults();                // the config.h values
bool   settingsLoad(Tuning& out);         // false if nothing valid is saved
bool   settingsSave(const Tuning& t);
void   settingsErase();

// Sensor calibration (min/max per sensor). Ignored if the sensor pins or
// count in config.h changed since it was saved.
bool settingsLoadCal(SensorCal& out);
bool settingsSaveCal(const SensorCal& c);

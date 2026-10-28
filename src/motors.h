#pragma once
// DRV8833 dual H-bridge driver. Speed is a signed fraction of the motor
// voltage limit: +1 = full forward at the limit, -1 = full reverse. The limit
// is MOTOR_RATED_V unless unlocked from the phone (up to MOTOR_UNLOCK_MAX_V),
// and PWM duty = speed x limit / supply voltage, so the motors never see
// more than the limit. The balance trim then slows the faster motor.

void  motorsBegin();
void  motorsSet(float left, float right);
void  motorsStop();  // active brake on both motors

// Balance trim: > 0 slows the right motor by that fraction, < 0 slows the
// left. Starts at MOTOR_TRIM.
void  motorsSetTrim(float trim);

// Dev unlock: motor voltage at speed 1.0, clamped to MOTOR_RATED_V ..
// MOTOR_UNLOCK_MAX_V. Safe to call from any core. Starts at MOTOR_RATED_V.
void  motorsSetVoltageLimit(float volts);
float motorsVoltageLimit();

// Motor supply (DRV8833 VM) the duty is worked out from: the measured
// battery voltage on PcbChassis. NaN or anything outside MOTOR_SUPPLY_MIN_V ..
// MOTOR_SUPPLY_MAX_V means MOTOR_SUPPLY_V (a full 2S pack), the start value.
void  motorsSetSupplyVoltage(float volts);
float motorsSupplyVoltage();

float motorsLeftDuty();   // last applied speed (after clamp and trim), for telemetry
float motorsRightDuty();

#pragma once
// Wheel encoders (PcbChassis): the quadrature hall encoders on the N20 motors,
// counted by the ESP32-S3 pulse counter (PCNT) hardware with x4 decoding, so
// counting costs the CPU nothing. Positive = the wheel turning forward
// (ENC_*_INVERT fix the sign). Nothing here on boards without encoders.
//
// The hardware counters are 16-bit and wrap at ENC_PCNT_LIMIT; encodersUpdate()
// folds them into 32-bit totals, so call it often (every control tick): the
// wheels must turn less than half the limit between calls.

#include <stdint.h>

void    encodersBegin();
void    encodersUpdate();          // one caller only (the control task or a test loop)
int32_t encoderCount(bool right);  // total counts since boot; safe from any core
float   encoderSpeedMms(bool right);  // over the last ENC_SPEED_WINDOW_MS; 0 until ENC_COUNTS_PER_REV is set

#pragma once
// Lap marker detection and lap timing, run from the control loop.
// The marker is a Cross tile line across the whole sensor array. The first
// crossing starts the clock; every LAP_CROSSINGS_PER_LAP crossings after
// that complete a lap.

#include <stdint.h>

struct LapEvent {
    uint16_t lap;      // laps completed after this event; 0 = timing just started
    uint32_t lapMs;    // time of the lap just completed (0 for the start event)
    uint32_t startMs;  // robot clock (millis) when the current lap began
};

void lapTimerBegin();

// Call every control tick while the robot is running. Returns true and fills
// ev when a start/finish crossing is accepted.
bool lapTimerUpdate(uint32_t sensorBits, LapEvent& ev);

uint16_t lapCount();   // laps completed
uint32_t lastLapMs();  // 0 until the first lap completes

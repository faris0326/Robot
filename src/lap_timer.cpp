#include "lap_timer.h"

#include <Arduino.h>

#include "config.h"

static_assert(LAP_MIN_SENSORS >= 3 && LAP_MIN_SENSORS <= IR_COUNT,
              "the line alone can cover 2 sensors, so the marker needs at least 3");
static_assert(LAP_CROSSINGS_PER_LAP >= 1, "at least one crossing per lap");

namespace {

constexpr uint32_t WINDOW_US = LAP_WINDOW_MS * 1000;

bool     started = false;
uint16_t laps = 0;
uint32_t lastLap = 0;
uint8_t  crossings = 0;     // crossings since the current lap started
uint32_t lapStartUs = 0;
uint32_t lastCrossUs = 0;
uint32_t lockoutUs = LAP_MIN_LOCKOUT_MS * 1000;

uint32_t lastBlackUs[IR_COUNT] = {};
bool     onMarker = false;  // stays true until the window no longer sees the marker

}  // namespace

void lapTimerBegin() {
    started = false;
    laps = 0;
    lastLap = 0;
    crossings = 0;
    onMarker = false;
    lockoutUs = LAP_MIN_LOCKOUT_MS * 1000;
}

bool lapTimerUpdate(uint32_t sensorBits, LapEvent& ev) {
    uint32_t nowUs = micros();

    uint8_t recent = 0;
    for (uint8_t i = 0; i < IR_COUNT; i++) {
        if (sensorBits & (1u << i)) lastBlackUs[i] = nowUs;
        if (nowUs - lastBlackUs[i] <= WINDOW_US) recent++;
    }

    if (recent < LAP_MIN_SENSORS) {
        onMarker = false;
        return false;
    }
    if (onMarker) return false;  // same crossing, already handled
    onMarker = true;

    // The crossing is timed when the last needed sensor reaches the line.
    if (started && nowUs - lastCrossUs < lockoutUs) return false;
    lastCrossUs = nowUs;

    if (!started) {
        started = true;
        lapStartUs = nowUs;
        crossings = 0;
        ev = {0, 0, millis()};
        return true;
    }

    if (++crossings < LAP_CROSSINGS_PER_LAP) return false;
    crossings = 0;

    uint32_t lapUs = nowUs - lapStartUs;
    lapStartUs = nowUs;
    lastLap = (lapUs + 500) / 1000;
    laps++;
    lockoutUs = max<uint32_t>(LAP_MIN_LOCKOUT_MS * 1000,
                              lapUs * LAP_LOCKOUT_FRACTION / LAP_CROSSINGS_PER_LAP);

    ev = {laps, lastLap, millis()};
    return true;
}

uint16_t lapCount()  { return laps; }
uint32_t lastLapMs() { return lastLap; }

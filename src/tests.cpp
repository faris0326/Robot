#include "tests.h"

#include <Arduino.h>

#include "config.h"
#include "control.h"
#include "motors.h"
#include "sensors.h"

namespace {

void step(const char* label, float l, float r, uint32_t ms) {
    Serial.printf("%-22s L=%+.2f R=%+.2f\n", label, l, r);
    motorsSet(l, r);
    delay(ms);
    motorsStop();
    delay(400);
}

void ramp(float to, uint32_t ms) {
    Serial.printf("ramp both 0 -> %+.2f\n", to);
    const int steps = 50;
    for (int i = 0; i <= steps; i++) {
        float d = to * i / steps;
        motorsSet(d, d);
        delay(ms / steps);
    }
    motorsStop();
    delay(400);
}

}  // namespace

void motorTest() {
    Serial.printf("\nMotor test. Full speed = %.1f V (PWM duty %.2f), starting in %lu ms\n",
                  MOTOR_RATED_V, MAX_DUTY, (unsigned long)START_DELAY_MS);
    delay(START_DELAY_MS);

    const float d = 0.5f;
    step("left forward",   d, 0, 1500);
    step("left reverse",  -d, 0, 1500);
    step("right forward",  0, d, 1500);
    step("right reverse",  0, -d, 1500);
    step("both forward",   d, d, 1500);
    step("spin (L+, R-)",  d, -d, 1500);
    ramp(1.0f, 2000);  // up to full rated voltage

    Serial.println("done, motors braked");
}

namespace {

// Left of the row is O1 (left side of the robot).
// One character per sensor: '.' = background, '1'..'9' = how much line it sees.
void formatBits(char* row) {
    uint8_t lv[IR_COUNT];
    sensorsLevels(lv);
    for (uint8_t i = 0; i < IR_COUNT; i++) row[i] = lv[i] < 26 ? '.' : (char)('0' + min(9, lv[i] * 10 / 256));
    row[IR_COUNT] = '\0';
}

const char* stateLabel() {
    switch (lineState()) {
        case LineState::InGap: return "  GAP (straight)";
        case LineState::Lost:  return "  LOST";
        default:               return "";
    }
}

}  // namespace

// Prints the line levels, the position and each sensor's raw ADC value. The
// leftmost character is D1; it should be the robot's left side.
void sensorTestLoop() {
    float pos = readPosition();
    char row[IR_COUNT + 1];
    formatBits(row);
    uint16_t raw[IR_COUNT];
    sensorsRaw(raw);

    Serial.printf("[%s]  pos=%+.2f%-6s raw", row, pos, stateLabel());
    for (uint8_t i = 0; i < IR_COUNT; i++) Serial.printf(" %4u", raw[i]);
    Serial.println();
    delay(100);
}

void steerTestLoop() {
    static uint8_t n = 0;

    float pos = readPosition();
    float l, r;
    controlUpdate(pos, 0.01f, l, r);
    motorsSet(l, r);

    if (++n >= 10) {
        n = 0;
        char row[IR_COUNT + 1];
        formatBits(row);
        Serial.printf("[%s]  pos=%+.2f  L=%+.2f R=%+.2f%s\n", row, pos,
                      motorsLeftDuty(), motorsRightDuty(), stateLabel());
    }
    delay(10);
}

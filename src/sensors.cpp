#include "sensors.h"

#include <Arduino.h>

namespace {

// ADC2 (GPIO 11..20) is shared with Wi-Fi and fails while it runs, and
// GPIO 3 is a strapping pin. Only ADC1 pins can carry the sensors.
constexpr bool isAdc1(uint8_t pin) { return pin >= 1 && pin <= 10 && pin != 3; }
constexpr bool allAdc1(uint8_t i = 0) {
    return i >= IR_COUNT || (isAdc1(PIN_IR[i]) && allAdc1(i + 1));
}

}  // namespace

static_assert(IR_COUNT <= 16, "telemetry carries at most 16 sensor levels");
static_assert(allAdc1(), "IR sensor pins must be ADC1 (GPIO 1..10, not 3): ADC2 can't be read while Wi-Fi is on");

namespace {

uint16_t  raw[IR_COUNT];
float     level[IR_COUNT];
uint32_t  bits = 0;
LineState state = LineState::Lost;
float     lastSeen = 0.0f;
float     weightScale = 1.0f;  // maps the outermost weight to +/-1

// The calibration is used on core 1 and copied for saving on core 0.
portMUX_TYPE calMux = portMUX_INITIALIZER_UNLOCKED;
SensorCal    cal;
SensorCal    calRun;          // min/max being recorded during a calibration
bool         calActive = false;

}  // namespace

void sensorsBegin() {
    pinMode(PIN_IR_LED, OUTPUT);
    digitalWrite(PIN_IR_LED, HIGH);  // emitters on

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // ~0..3.1 V input range

    float maxWeight = 0.0f;
    for (uint8_t i = 0; i < IR_COUNT; i++) {
        pinMode(PIN_IR[i], INPUT);
        maxWeight = max(maxWeight, fabsf(IR_WEIGHTS[i]));
        cal.min[i] = IR_DEFAULT_MIN;
        cal.max[i] = IR_DEFAULT_MAX;
    }
    weightScale = maxWeight > 0.0f ? 1.0f / maxWeight : 1.0f;
}

float readPosition() {
    for (uint8_t i = 0; i < IR_COUNT; i++) raw[i] = analogRead(PIN_IR[i]);

    if (calActive) {
        for (uint8_t i = 0; i < IR_COUNT; i++) {
            calRun.min[i] = min(calRun.min[i], raw[i]);
            calRun.max[i] = max(calRun.max[i], raw[i]);
        }
    }

    bits = 0;
    float sum = 0.0f, total = 0.0f, best = 0.0f;
    for (uint8_t i = 0; i < IR_COUNT; i++) {
        float span = max(1, (int)cal.max[i] - (int)cal.min[i]);
        // Raw is high over black, so for a white line the level is inverted.
        float v = constrain(((int)raw[i] - (int)cal.min[i]) / span, 0.0f, 1.0f);
        if (LINE_IS_WHITE) v = 1.0f - v;
        level[i] = v;

        if (v > IR_ON_LINE) bits |= 1u << i;
        best = max(best, v);
        if (v > IR_NOISE_FLOOR) {
            sum += v * IR_WEIGHTS[i];
            total += v;
        }
    }

    if (best < IR_ON_LINE) {
        state = LineState::Lost;
        if (lastSeen < 0.0f) return -1.0f;
        if (lastSeen > 0.0f) return 1.0f;
        return 0.0f;
    }

    state = LineState::OnLine;
    lastSeen = constrain(sum / total * weightScale, -1.0f, 1.0f);
    return lastSeen;
}

uint32_t  sensorsBits() { return bits; }
LineState lineState()   { return state; }

void sensorsLevels(uint8_t* out) {
    for (uint8_t i = 0; i < IR_COUNT; i++) out[i] = (uint8_t)lroundf(level[i] * 255.0f);
}

void sensorsRaw(uint16_t* out) {
    for (uint8_t i = 0; i < IR_COUNT; i++) out[i] = raw[i];
}

SensorCal sensorsCalibration() {
    portENTER_CRITICAL(&calMux);
    SensorCal c = cal;
    portEXIT_CRITICAL(&calMux);
    return c;
}

void sensorsSetCalibration(const SensorCal& c) {
    portENTER_CRITICAL(&calMux);
    cal = c;
    portEXIT_CRITICAL(&calMux);
}

void sensorsCalibrateBegin() {
    for (uint8_t i = 0; i < IR_COUNT; i++) {
        calRun.min[i] = 4095;
        calRun.max[i] = 0;
    }
    calActive = true;
}

bool sensorsCalibrateEnd(int8_t& badSensor) {
    calActive = false;
    badSensor = -1;
    for (uint8_t i = 0; i < IR_COUNT; i++) {
        if (calRun.max[i] < calRun.min[i] + IR_CAL_MIN_RANGE) {
            badSensor = i;
            return false;
        }
    }
    sensorsSetCalibration(calRun);
    return true;
}

void sensorsCalibrateAbort() { calActive = false; }

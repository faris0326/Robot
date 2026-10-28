#include "sensors.h"

#include <Arduino.h>
#include <string.h>

#include <atomic>

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
int8_t    lastSide = 0;    // -1 / +1: side the line was last clearly seen on
int8_t    corner = 0;      // -1 / +1: a corner to that side is under the array

// Line colour in use (core 1) and the one asked for from the phone (core 0).
// readPosition() applies a change, so only core 1 ever rewrites `cal`.
bool              lineWhite = LINE_IS_WHITE;
std::atomic<bool> wantWhite{LINE_IS_WHITE};
float     weightScale = 1.0f;  // maps the outermost weight to +/-1

// The calibration is used on core 1 and copied for saving on core 0.
portMUX_TYPE calMux = portMUX_INITIALIZER_UNLOCKED;
SensorCal    cal;

// Readings recorded during a calibration run (core 1 only): a histogram per
// sensor, CAL_BIN_SHIFT = 5 -> 32 raw counts (~25 mV) per bin.
constexpr uint8_t CAL_BIN_SHIFT = 5;
constexpr uint16_t CAL_BINS = 4096 >> CAL_BIN_SHIFT;
uint16_t calHist[IR_COUNT][CAL_BINS];
bool     calActive = false;

// One sensor's readings split into a low and a high group.
struct Split {
    float lowRaw, highRaw;  // average raw reading of each group
};

// Otsu's method: the split point that best separates the histogram into two
// groups (largest between-group variance). Single stray readings barely move
// the group averages, unlike a plain min/max, and each group must hold at
// least 1/CAL_MIN_GROUP of the samples (24 ms of a 2.4 s spin), so a few
// stray readings can't pose as the line.
constexpr uint32_t CAL_MIN_GROUP = 100;

Split splitHistogram(const uint16_t* h) {
    uint32_t n = 0;
    float    sum = 0.0f;
    for (uint16_t b = 0; b < CAL_BINS; b++) {
        n += h[b];
        sum += (float)h[b] * b;
    }

    Split best = {0, 0};
    float bestVar = -1.0f, lowSum = 0.0f;
    uint32_t lowN = 0;
    for (uint16_t t = 1; t < CAL_BINS; t++) {  // low group = bins below t
        lowN += h[t - 1];
        lowSum += (float)h[t - 1] * (t - 1);
        uint32_t highN = n - lowN;
        uint32_t minN = max<uint32_t>(1, n / CAL_MIN_GROUP);
        if (lowN < minN || highN < minN) continue;
        float lowMean = lowSum / lowN, highMean = (sum - lowSum) / highN;
        float d = highMean - lowMean;
        float var = (float)lowN * highN * d * d;
        if (var > bestVar) {
            bestVar = var;
            // Bin centre -> raw ADC value.
            best = {(lowMean + 0.5f) * (1 << CAL_BIN_SHIFT), (highMean + 0.5f) * (1 << CAL_BIN_SHIFT)};
        }
    }
    return best;
}

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
        cal.background[i] = LINE_IS_WHITE ? IR_DEFAULT_BLACK : IR_DEFAULT_WHITE;
        cal.line[i]       = LINE_IS_WHITE ? IR_DEFAULT_WHITE : IR_DEFAULT_BLACK;
    }
    weightScale = maxWeight > 0.0f ? 1.0f / maxWeight : 1.0f;
}

float readPosition() {
    if (wantWhite != lineWhite) {
        lineWhite = wantWhite;
        sensorsSetCalibration(sensorsCalibration());  // swaps line/background
    }

    for (uint8_t i = 0; i < IR_COUNT; i++) raw[i] = analogRead(PIN_IR[i]);

    if (calActive) {
        for (uint8_t i = 0; i < IR_COUNT; i++) {
            uint16_t& c = calHist[i][raw[i] >> CAL_BIN_SHIFT];
            if (c < UINT16_MAX) c++;
        }
    }

    bits = 0;
    float sum = 0.0f, total = 0.0f, best = 0.0f;
    for (uint8_t i = 0; i < IR_COUNT; i++) {
        // Negative span = the line reads lower than the background (white line).
        float span = (int)cal.line[i] - (int)cal.background[i];
        if (fabsf(span) < 1.0f) span = 1.0f;
        float v = constrain(((int)raw[i] - (int)cal.background[i]) / span, 0.0f, 1.0f);
        level[i] = v;

        if (v > IR_ON_LINE) bits |= 1u << i;
        best = max(best, v);
        if (v > IR_NOISE_FLOOR) {
            sum += v * IR_WEIGHTS[i];
            total += v;
        }
    }

    corner = 0;
    if (best < IR_ON_LINE) {
        state = LineState::Lost;
        if (lastSide != 0) return lastSide;
        if (lastSeen < 0.0f) return -1.0f;
        if (lastSeen > 0.0f) return 1.0f;
        return 0.0f;
    }

    state = LineState::OnLine;
    lastSeen = constrain(sum / total * weightScale, -1.0f, 1.0f);

    // Which way the line went. At a 90-degree corner the side branch reaches
    // one outer sensor, then the line can vanish under the middle sensors
    // last, so the sign of the very last position would be a guess.
    bool leftEdge = level[0] > IR_ON_LINE, rightEdge = level[IR_COUNT - 1] > IR_ON_LINE;
    if (leftEdge != rightEdge) lastSide = leftEdge ? -1 : 1;
    else if (fabsf(lastSeen) > LOST_SIDE_MIN) lastSide = lastSeen < 0.0f ? -1 : 1;

    // A corner's side branch makes the line wide and reaches one outer sensor.
    if (leftEdge != rightEdge && __builtin_popcount(bits) >= CORNER_MIN_SENSORS) corner = lastSide;
    return lastSeen;
}

uint32_t  sensorsBits() { return bits; }
int8_t    sensorsCorner() { return corner; }
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

void sensorsSetCalibration(const SensorCal& in) {
    // Raw is high over black: a black line must read above its background.
    SensorCal c = in;
    for (uint8_t i = 0; i < IR_COUNT; i++) {
        if ((c.line[i] < c.background[i]) != lineWhite) {
            uint16_t t = c.line[i];
            c.line[i] = c.background[i];
            c.background[i] = t;
        }
    }
    portENTER_CRITICAL(&calMux);
    cal = c;
    portEXIT_CRITICAL(&calMux);
}

void sensorsCalibrateBegin() {
    memset(calHist, 0, sizeof calHist);
    calActive = true;
}

bool sensorsCalibrateEnd(int8_t& badSensor) {
    calActive = false;
    badSensor = -1;

    Split split[IR_COUNT];
    for (uint8_t i = 0; i < IR_COUNT; i++) {
        split[i] = splitHistogram(calHist[i]);
        if (split[i].highRaw - split[i].lowRaw < IR_CAL_MIN_CONTRAST) {
            badSensor = i;
            return false;
        }
    }

    // Raw is high over black: the high group is a black line's reading.
    SensorCal c;
    for (uint8_t i = 0; i < IR_COUNT; i++) {
        uint16_t lo = (uint16_t)lroundf(split[i].lowRaw), hi = (uint16_t)lroundf(split[i].highRaw);
        c.background[i] = lineWhite ? hi : lo;
        c.line[i]       = lineWhite ? lo : hi;
    }
    sensorsSetCalibration(c);
    return true;
}

void sensorsCalibrateAbort() { calActive = false; }

void sensorsSetLineWhite(bool white) { wantWhite = white; }
bool sensorsLineWhite() { return wantWhite; }

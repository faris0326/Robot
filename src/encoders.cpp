#include "encoders.h"

#include <Arduino.h>
#include <driver/pcnt.h>

#include <atomic>

#include "config.h"

namespace {

struct Wheel {
    pcnt_unit_t unit;
    uint8_t     pinA, pinB;  // C1, C2
    bool        invert;
};
const Wheel WHEELS[2] = {
    {PCNT_UNIT_0, PIN_ENC_L_C1, PIN_ENC_L_C2, ENC_L_INVERT},
    {PCNT_UNIT_1, PIN_ENC_R_C1, PIN_ENC_R_C2, ENC_R_INVERT},
};

std::atomic<int32_t> total[2];
std::atomic<float>   speed[2];
int16_t  lastRaw[2] = {0, 0};
int32_t  windowStart[2] = {0, 0};
uint32_t windowMs = 0;
bool     started = false;

// Both channels of a unit together give x4 quadrature: each counts the edges
// of one signal, in a direction set by the level of the other.
void setupUnit(const Wheel& w) {
    pcnt_config_t c = {};
    c.unit = w.unit;
    c.counter_h_lim = ENC_PCNT_LIMIT;
    c.counter_l_lim = -ENC_PCNT_LIMIT;

    c.channel = PCNT_CHANNEL_0;
    c.pulse_gpio_num = w.pinA;
    c.ctrl_gpio_num = w.pinB;
    c.pos_mode = PCNT_COUNT_DEC;
    c.neg_mode = PCNT_COUNT_INC;
    c.lctrl_mode = PCNT_MODE_REVERSE;
    c.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_unit_config(&c);

    c.channel = PCNT_CHANNEL_1;
    c.pulse_gpio_num = w.pinB;
    c.ctrl_gpio_num = w.pinA;
    c.pos_mode = PCNT_COUNT_INC;
    c.neg_mode = PCNT_COUNT_DEC;
    pcnt_unit_config(&c);

    pcnt_set_filter_value(w.unit, ENC_GLITCH_FILTER);  // APB cycles (12.5 ns each)
    pcnt_filter_enable(w.unit);
    pcnt_counter_pause(w.unit);
    pcnt_counter_clear(w.unit);
    pcnt_counter_resume(w.unit);
}

}  // namespace

void encodersBegin() {
    if (!ENCODERS_FITTED) return;
    for (const Wheel& w : WHEELS) setupUnit(w);
    windowMs = millis();
    started = true;
}

void encodersUpdate() {
    if (!started) return;
    for (int i = 0; i < 2; i++) {
        int16_t raw = 0;
        pcnt_get_counter_value(WHEELS[i].unit, &raw);
        // The counter jumps back to 0 at +/-ENC_PCNT_LIMIT: count modulo the limit.
        int32_t d = (int32_t)raw - lastRaw[i];
        if (d > ENC_PCNT_LIMIT / 2) d -= ENC_PCNT_LIMIT;
        else if (d < -ENC_PCNT_LIMIT / 2) d += ENC_PCNT_LIMIT;
        lastRaw[i] = raw;
        total[i] += WHEELS[i].invert ? -d : d;
    }

    uint32_t now = millis();
    uint32_t elapsed = now - windowMs;
    if (elapsed < ENC_SPEED_WINDOW_MS) return;
    float mmPerCount = ENC_COUNTS_PER_REV > 0.0f ? PI * WHEEL_DIAMETER_MM / ENC_COUNTS_PER_REV : 0.0f;
    for (int i = 0; i < 2; i++) {
        int32_t t = total[i];
        speed[i] = (t - windowStart[i]) * mmPerCount * 1000.0f / elapsed;
        windowStart[i] = t;
    }
    windowMs = now;
}

int32_t encoderCount(bool right) { return total[right ? 1 : 0]; }
float   encoderSpeedMms(bool right) { return speed[right ? 1 : 0]; }

#include "motors.h"

#include <Arduino.h>

#include "config.h"

namespace {

constexpr uint32_t PWM_MAX = (1u << MOTOR_PWM_BITS) - 1;

// LEDC channels (only used by the 2.x core API)
constexpr uint8_t CH_L_IN1 = 0;
constexpr uint8_t CH_L_IN2 = 1;
constexpr uint8_t CH_R_IN1 = 2;
constexpr uint8_t CH_R_IN2 = 3;

struct Motor {
    uint8_t pin1, pin2;
    uint8_t ch1, ch2;
    bool invert;
    float gain;   // balance trim, 0..1
    float speed;  // last applied, after trim, -1..+1 of rated voltage
};

Motor left  = {PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, CH_L_IN1, CH_L_IN2, MOTOR_L_INVERT, 1.0f, 0.0f};
Motor right = {PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, CH_R_IN1, CH_R_IN2, MOTOR_R_INVERT, 1.0f, 0.0f};

void pwmAttach(uint8_t pin, uint8_t ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)ch;
    ledcAttach(pin, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
#else
    ledcSetup(ch, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
    ledcAttachPin(pin, ch);
#endif
}

void pwmWrite(uint8_t pin, uint8_t ch, uint32_t value) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)ch;
    ledcWrite(pin, value);
#else
    (void)pin;
    ledcWrite(ch, value);
#endif
}

void apply(Motor& m, float speed) {
    speed = constrain(speed, -1.0f, 1.0f) * m.gain;
    m.speed = speed;

    // Speed 1.0 = rated motor voltage = MAX_DUTY of the supply.
    float d = (m.invert ? -speed : speed) * MAX_DUTY;
    bool forward = d >= 0.0f;
    uint32_t mag = (uint32_t)(fabsf(d) * PWM_MAX + 0.5f);

    // DRV8833 truth table:
    //   fast decay: drive pin = PWM, other pin = 0 (coast between pulses)
    //   slow decay: other pin = 1, drive pin = inverted PWM (brake between pulses)
    uint32_t hi, lo;
    if (MOTOR_SLOW_DECAY) {
        hi = PWM_MAX;
        lo = PWM_MAX - mag;
    } else {
        hi = mag;
        lo = 0;
    }

    if (forward) {
        pwmWrite(m.pin1, m.ch1, hi);
        pwmWrite(m.pin2, m.ch2, lo);
    } else {
        pwmWrite(m.pin1, m.ch1, lo);
        pwmWrite(m.pin2, m.ch2, hi);
    }
}

void brake(Motor& m) {
    m.speed = 0.0f;
    pwmWrite(m.pin1, m.ch1, PWM_MAX);
    pwmWrite(m.pin2, m.ch2, PWM_MAX);
}

}  // namespace

void motorsBegin() {
    pwmAttach(left.pin1, left.ch1);
    pwmAttach(left.pin2, left.ch2);
    pwmAttach(right.pin1, right.ch1);
    pwmAttach(right.pin2, right.ch2);
    motorsSetTrim(MOTOR_TRIM);
    motorsStop();
}

void motorsSetTrim(float trim) {
    trim = constrain(trim, -0.9f, 0.9f);
    left.gain = trim < 0.0f ? 1.0f + trim : 1.0f;
    right.gain = trim > 0.0f ? 1.0f - trim : 1.0f;
}

void motorsSet(float l, float r) {
    apply(left, l);
    apply(right, r);
}

void motorsStop() {
    brake(left);
    brake(right);
}

float motorsLeftDuty()  { return left.speed; }
float motorsRightDuty() { return right.speed; }

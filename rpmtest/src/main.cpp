// Encoder / RPM test for the line follower (ESP32-S3, Arduino core 2.0.17).
//
// Purpose: measure the real wheel speed so we can choose a gear ratio.
//
//   1. Calibration mode (default at boot): motors braked, prints the raw
//      left/right encoder counts every 200 ms. Type "z" to zero them, turn a
//      wheel exactly one revolution by hand (or 10 and divide by 10 for a
//      more accurate number) and read the counts per wheel revolution.
//   2. RPM test: type "cpr 700" (or "cpr 700 704" for different left/right
//      values) once, then "run 50" spins both wheels forward at 50 % duty for
//      3 s and prints RPM and m/s every 200 ms, then the average and peak.
//      "sweep" runs the duty steps 30, 50, 70, 75 % one after another and
//      prints a summary table.
//
// Commands (type and press Enter, "help" lists them):
//   z            zero the counters
//   cal          back to calibration mode (counts every 200 ms)
//   cpr N [N2]   counts per wheel revolution (left [right])
//   run P        run both motors at P % duty (capped at 75 %)
//   sweep        run 30, 50, 70, 75 % with a summary table
//   dur MS       change the run length (default 3000 ms)
//   s            stop (any input also aborts a running test)
//
// Encoders are decoded in full quadrature (x4): an interrupt on every edge of
// both channels, so counts per revolution = 4 x encoder pulses per motor
// revolution x gear ratio.

#include <Arduino.h>

#include "soc/gpio_reg.h"
#include "soc/soc.h"

// ===========================================================================
// Pins and constants (same pin map as the robot, see ../include/config.h)
// ===========================================================================

// DRV8833: right motor IN1/IN2, left motor IN3/IN4
constexpr uint8_t PIN_MOTOR_R_IN1 = 18;
constexpr uint8_t PIN_MOTOR_R_IN2 = 17;
constexpr uint8_t PIN_MOTOR_L_IN1 = 16;  // DRV8833 IN3
constexpr uint8_t PIN_MOTOR_L_IN2 = 15;  // DRV8833 IN4

// The left motor runs backwards on a forward command (found with steertest).
constexpr bool MOTOR_L_INVERT = true;
constexpr bool MOTOR_R_INVERT = false;

// Encoder channels C1 (A) and C2 (B)
constexpr uint8_t PIN_ENC_L_A = 13;
constexpr uint8_t PIN_ENC_L_B = 14;
constexpr uint8_t PIN_ENC_R_A = 12;
constexpr uint8_t PIN_ENC_R_B = 11;

// The ISR reads all pins at once from GPIO_IN_REG, which covers GPIO 0..31.
static_assert(PIN_ENC_L_A < 32 && PIN_ENC_L_B < 32 && PIN_ENC_R_A < 32 && PIN_ENC_R_B < 32,
              "encoder pins must be GPIO 0..31");

constexpr uint32_t PWM_FREQ_HZ = 20000;  // above hearing range
constexpr uint8_t  PWM_BITS    = 10;     // 0..1023
constexpr uint32_t PWM_MAX     = (1u << PWM_BITS) - 1;
constexpr int      DUTY_CAP_PCT = 75;    // hard limit for this test

constexpr float WHEEL_DIAMETER_M = 0.0335f;

constexpr uint32_t PRINT_PERIOD_MS = 200;
constexpr uint32_t RUN_MS_DEFAULT  = 3000;
// Samples that end before this are spin-up and are left out of the average.
constexpr uint32_t SETTLE_MS       = 1000;
constexpr uint32_t SWEEP_REST_MS   = 1500;  // let the wheels stop between steps

constexpr int SWEEP_STEPS[] = {30, 50, 70, 75};
constexpr int SWEEP_COUNT = sizeof(SWEEP_STEPS) / sizeof(SWEEP_STEPS[0]);

// ===========================================================================
// Quadrature decoding
// ===========================================================================

struct Encoder {
    uint8_t pinA, pinB;
    volatile int32_t count;
    volatile uint8_t prev;  // last (A << 1 | B)
};

Encoder encL = {PIN_ENC_L_A, PIN_ENC_L_B, 0, 0};
Encoder encR = {PIN_ENC_R_A, PIN_ENC_R_B, 0, 0};

// Index = (previous AB << 2) | new AB. A valid Gray-code step gives +1 or -1;
// no change or a double step (missed edge / noise) gives 0.
DRAM_ATTR static const int8_t QDEC[16] = {
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0,
};

static inline uint8_t IRAM_ATTR readAB(const Encoder& e) {
    uint32_t in = REG_READ(GPIO_IN_REG);
    return (uint8_t)((((in >> e.pinA) & 1u) << 1) | ((in >> e.pinB) & 1u));
}

static inline void IRAM_ATTR encUpdate(Encoder& e) {
    uint8_t ab = readAB(e);
    e.count = e.count + QDEC[(e.prev << 2) | ab];
    e.prev = ab;
}

void IRAM_ATTR isrLeft()  { encUpdate(encL); }
void IRAM_ATTR isrRight() { encUpdate(encR); }

void encodersBegin() {
    // INPUT_PULLUP in case the encoder outputs are open collector.
    for (uint8_t pin : {PIN_ENC_L_A, PIN_ENC_L_B, PIN_ENC_R_A, PIN_ENC_R_B}) pinMode(pin, INPUT_PULLUP);
    encL.prev = readAB(encL);
    encR.prev = readAB(encR);
    attachInterrupt(PIN_ENC_L_A, isrLeft, CHANGE);
    attachInterrupt(PIN_ENC_L_B, isrLeft, CHANGE);
    attachInterrupt(PIN_ENC_R_A, isrRight, CHANGE);
    attachInterrupt(PIN_ENC_R_B, isrRight, CHANGE);
}

// "z" never writes the counters (the ISR owns them); it stores an offset.
int32_t zeroL = 0, zeroR = 0;

int32_t countL() { return encL.count - zeroL; }
int32_t countR() { return encR.count - zeroR; }
void countsZero() { zeroL = encL.count; zeroR = encR.count; }

// ===========================================================================
// Motors (DRV8833, slow decay like the robot firmware)
// ===========================================================================

struct Motor {
    uint8_t pin1, pin2, ch1, ch2;
    bool invert;
};

Motor motorL = {PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, 0, 1, MOTOR_L_INVERT};
Motor motorR = {PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, 2, 3, MOTOR_R_INVERT};

void motorAttach(const Motor& m) {
    ledcSetup(m.ch1, PWM_FREQ_HZ, PWM_BITS);
    ledcSetup(m.ch2, PWM_FREQ_HZ, PWM_BITS);
    ledcAttachPin(m.pin1, m.ch1);
    ledcAttachPin(m.pin2, m.ch2);
}

// duty -1..1 (+ = forward), clamped to DUTY_CAP_PCT.
// Slow decay: one pin held high, the other gets inverted PWM (brake between
// pulses), which gives a more linear speed vs duty curve.
void motorSet(const Motor& m, float duty) {
    const float cap = DUTY_CAP_PCT / 100.0f;
    duty = constrain(duty, -cap, cap);
    if (m.invert) duty = -duty;
    uint32_t mag = (uint32_t)(fabsf(duty) * PWM_MAX + 0.5f);
    uint32_t hi = PWM_MAX, lo = PWM_MAX - mag;
    if (duty >= 0.0f) {
        ledcWrite(m.ch1, hi);
        ledcWrite(m.ch2, lo);
    } else {
        ledcWrite(m.ch1, lo);
        ledcWrite(m.ch2, hi);
    }
}

void motorsSet(float duty) { motorSet(motorL, duty); motorSet(motorR, duty); }

void motorsBrake() {
    for (const Motor* m : {&motorL, &motorR}) {
        ledcWrite(m->ch1, PWM_MAX);
        ledcWrite(m->ch2, PWM_MAX);
    }
}

// ===========================================================================
// Speed maths
// ===========================================================================

float cprL = 0, cprR = 0;  // counts per wheel revolution, 0 = not set yet
uint32_t runMs = RUN_MS_DEFAULT;

float rpmFrom(int32_t counts, float cpr, uint32_t dtUs) {
    return fabsf((float)counts) / cpr * 60e6f / (float)dtUs;
}

float mpsFrom(float rpm) { return PI * WHEEL_DIAMETER_M * rpm / 60.0f; }

// ===========================================================================
// Serial input
// ===========================================================================

// Collects characters until Enter. Returns true when a full line is in buf.
bool readLine(char* buf, size_t size) {
    static size_t len = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            if (len == 0) continue;
            buf[len] = '\0';
            len = 0;
            return true;
        }
        if (len < size - 1) buf[len++] = c;
    }
    return false;
}

// ===========================================================================
// RPM run
// ===========================================================================

struct Result {
    int pct;
    float avgL, avgR, peakL, peakR;
    bool valid;  // false if aborted or nothing was in the settled window
};

Result runTest(int pct) {
    Result res = {pct, 0, 0, 0, 0, false};
    pct = constrain(pct, 0, DUTY_CAP_PCT);
    res.pct = pct;

    Serial.printf("\n--- run %d %% for %lu ms (any input aborts) ---\n", pct, (unsigned long)runMs);
    Serial.println("   t ms |   L rpm    L m/s |   R rpm    R m/s");

    int32_t lastL = countL(), lastR = countR();
    int32_t startL = lastL, startR = lastR;
    uint32_t t0Us = micros(), lastUs = t0Us;
    uint32_t t0Ms = millis(), nextMs = t0Ms + PRINT_PERIOD_MS;

    float sumL = 0, sumR = 0;
    int settled = 0;
    bool aborted = false;

    while (Serial.available()) Serial.read();  // drop the Enter that started us
    motorsSet(pct / 100.0f);

    while (millis() - t0Ms < runMs) {
        if (Serial.available()) {
            aborted = true;
            break;
        }
        if ((int32_t)(millis() - nextMs) < 0) continue;
        nextMs += PRINT_PERIOD_MS;

        int32_t cL = countL(), cR = countR();
        uint32_t nowUs = micros();
        uint32_t dtUs = nowUs - lastUs;
        float rL = rpmFrom(cL - lastL, cprL, dtUs);
        float rR = rpmFrom(cR - lastR, cprR, dtUs);
        lastL = cL; lastR = cR; lastUs = nowUs;

        uint32_t tMs = (nowUs - t0Us) / 1000;
        bool inAvg = tMs > SETTLE_MS;
        Serial.printf("%7lu%c| %7.1f  %7.3f | %7.1f  %7.3f\n", (unsigned long)tMs, inAvg ? '*' : ' ',
                      rL, mpsFrom(rL), rR, mpsFrom(rR));

        res.peakL = max(res.peakL, rL);
        res.peakR = max(res.peakR, rR);
        if (inAvg) {
            sumL += rL;
            sumR += rR;
            settled++;
        }
    }
    motorsBrake();

    if (aborted) {
        while (Serial.available()) Serial.read();
        Serial.println("ABORTED, motors braked");
        return res;
    }

    if (countL() == startL) Serial.println("WARNING: no left encoder counts. Check wiring/power.");
    if (countR() == startR) Serial.println("WARNING: no right encoder counts. Check wiring/power.");

    if (settled == 0) {
        Serial.println("Run too short for an average (needs > SETTLE_MS).");
        return res;
    }
    res.avgL = sumL / settled;
    res.avgR = sumR / settled;
    res.valid = true;

    float avg = (res.avgL + res.avgR) / 2;
    Serial.printf("average (rows marked *): L %.1f rpm  R %.1f rpm  -> %.3f m/s\n", res.avgL, res.avgR,
                  mpsFrom(avg));
    Serial.printf("peak:                    L %.1f rpm  R %.1f rpm\n", res.peakL, res.peakR);
    if (avg > 0) Serial.printf("L/R difference: %+.1f %%\n", 100.0f * (res.avgL - res.avgR) / avg);
    return res;
}

void sweep() {
    Result results[SWEEP_COUNT];
    int done = 0;
    while (done < SWEEP_COUNT) {
        results[done] = runTest(SWEEP_STEPS[done]);
        if (!results[done].valid) {
            Serial.println("sweep stopped");
            break;
        }
        done++;
        if (done < SWEEP_COUNT) delay(SWEEP_REST_MS);
    }
    if (done == 0) return;

    Serial.printf("\n=== sweep summary (cpr L %.1f R %.1f, wheel %.1f mm) ===\n", cprL, cprR,
                  WHEEL_DIAMETER_M * 1000);
    Serial.println(" duty |  L avg  L peak |  R avg  R peak | avg m/s");
    for (int i = 0; i < done; i++) {
        const Result& r = results[i];
        Serial.printf("%3d %% | %6.1f  %6.1f | %6.1f  %6.1f | %6.3f\n", r.pct, r.avgL, r.peakL, r.avgR,
                      r.peakR, mpsFrom((r.avgL + r.avgR) / 2));
    }
}

// ===========================================================================
// Commands
// ===========================================================================

enum class Mode { Calibrate, Rpm };
Mode mode = Mode::Calibrate;

void printHelp() {
    Serial.println("\nCommands:");
    Serial.println("  z            zero the counters");
    Serial.println("  cal          calibration mode (counts every 200 ms, motors off)");
    Serial.println("  cpr N [N2]   counts per wheel revolution (left [right])");
    Serial.printf ("  run P        both motors at P %% duty (max %d) for the run length\n", DUTY_CAP_PCT);
    Serial.println("  sweep        run 30, 50, 70, 75 % with a summary table");
    Serial.printf ("  dur MS       run length in ms (now %lu)\n", (unsigned long)runMs);
    Serial.println("  s            stop printing / stop a run");
    Serial.println("  help         this list");
}

bool cprSet() {
    if (cprL > 0 && cprR > 0) return true;
    Serial.println("Set counts per revolution first, e.g. \"cpr 700\".");
    return false;
}

void handleCommand(char* line) {
    float a = 0, b = 0;
    int p = 0;
    unsigned long ms = 0;

    if (strcmp(line, "z") == 0) {
        countsZero();
        Serial.println("counters zeroed");
    } else if (strcmp(line, "cal") == 0) {
        motorsBrake();
        mode = Mode::Calibrate;
        Serial.println("calibration mode, motors braked");
    } else if (strcmp(line, "s") == 0) {
        motorsBrake();
        mode = Mode::Rpm;  // idle: no periodic printing
        Serial.println("stopped");
    } else if (strncmp(line, "cpr", 3) == 0) {
        int n = sscanf(line + 3, "%f %f", &a, &b);
        if (n < 1 || a == 0) {
            Serial.println("usage: cpr N  or  cpr LEFT RIGHT");
            return;
        }
        cprL = fabsf(a);
        cprR = n >= 2 && b != 0 ? fabsf(b) : cprL;
        Serial.printf("cpr left %.1f, right %.1f\n", cprL, cprR);
    } else if (sscanf(line, "run %d", &p) == 1) {
        if (!cprSet()) return;
        mode = Mode::Rpm;
        if (p > DUTY_CAP_PCT) Serial.printf("capped to %d %%\n", DUTY_CAP_PCT);
        runTest(p);
    } else if (strcmp(line, "sweep") == 0) {
        if (!cprSet()) return;
        mode = Mode::Rpm;
        sweep();
    } else if (sscanf(line, "dur %lu", &ms) == 1) {
        runMs = constrain(ms, 500UL, 10000UL);
        Serial.printf("run length %lu ms\n", (unsigned long)runMs);
    } else if (strcmp(line, "help") == 0 || strcmp(line, "h") == 0) {
        printHelp();
    } else {
        Serial.printf("unknown command \"%s\", type help\n", line);
    }
}

// ===========================================================================
// Arduino entry points
// ===========================================================================

void setup() {
    Serial.begin(115200);

    motorAttach(motorL);
    motorAttach(motorR);
    motorsBrake();  // outputs defined before anything else happens

    encodersBegin();

    delay(500);
    Serial.println("\nEncoder / RPM test. Motors braked.");
    printHelp();
    Serial.println("\nCalibration mode: type z, turn one wheel exactly one revolution, read the counts.");
}

void loop() {
    static char line[48];
    static uint32_t nextPrintMs = 0;

    if (readLine(line, sizeof(line))) {
        handleCommand(line);
        nextPrintMs = millis();
    }

    if (mode == Mode::Calibrate && (int32_t)(millis() - nextPrintMs) >= 0) {
        nextPrintMs = millis() + PRINT_PERIOD_MS;
        int32_t l = countL(), r = countR();
        // Raw A/B pin levels: if they never change while a wheel turns, the
        // encoder signal is not reaching the pin (power, wiring, pull-up).
        uint8_t abL = readAB(encL), abR = readAB(encR);
        if (cprL > 0 && cprR > 0)
            Serial.printf("L %8ld (%6.2f rev) AB=%d%d   R %8ld (%6.2f rev) AB=%d%d\n", (long)l, l / cprL,
                          abL >> 1, abL & 1, (long)r, r / cprR, abR >> 1, abR & 1);
        else
            Serial.printf("L %8ld AB=%d%d   R %8ld AB=%d%d\n", (long)l, abL >> 1, abL & 1, (long)r, abR >> 1,
                          abR & 1);
    }
}

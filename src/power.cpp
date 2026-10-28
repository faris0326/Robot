#include "power.h"

#include <Arduino.h>
#include <Wire.h>
#include <nvs.h>

#include "config.h"
#include "motors.h"

namespace {

// INA219 registers
constexpr uint8_t REG_CONFIG = 0x00;
constexpr uint8_t REG_SHUNT = 0x01;  // signed, 10 uV per bit
constexpr uint8_t REG_BUS = 0x02;    // bits 15..3, 4 mV per bit

// 16 V bus range, +/-320 mV shunt range (3.2 A with 0.1 ohm), 12-bit with
// 16-sample averaging on both channels (8.5 ms each, smooths the motor PWM),
// continuous conversion.
constexpr uint16_t INA_CONFIG = (0 << 13) | (3 << 11) | (0b1100 << 7) | (0b1100 << 3) | 0b111;

constexpr uint32_t PERIOD_MS = 20;
constexpr uint32_t RETRY_MS = 2000;       // INA219 missing: try again this often
constexpr float    AVG_TAU_S = 30.0f;     // time-left uses a ~30 s power average
constexpr uint32_t LOW_HOLD_MS = 2000;
constexpr float    LOW_MIN_A = 0.05f;     // only judge the rail while the battery supplies current
constexpr uint32_t SAVE_EVERY_MS = 60000;

constexpr char NVS_NAMESPACE[] = "battery";
constexpr char NVS_KEY[] = "usedmWh";

constexpr float USABLE_WH = BATTERY_MAH / 1000.0f * BATTERY_NOMINAL_V * BATTERY_USABLE;

portMUX_TYPE  mux = portMUX_INITIALIZER_UNLOCKED;
PowerStats    stats = {};
double        usedWh = 0.0;              // owned by the power task after powerBegin()
volatile bool resetRequested = false;   // telemetry -> power task
volatile bool saveNow = false;          // power task -> telemetry: save right away

// Flash bookkeeping, telemetry task only
float    savedWh = 0.0f;
uint32_t lastSaveMs = 0;

bool writeReg(uint8_t reg, uint16_t v) {
    Wire.beginTransmission(INA219_ADDR);
    Wire.write(reg);
    Wire.write(v >> 8);
    Wire.write(v & 0xFF);
    return Wire.endTransmission() == 0;
}

bool readReg(uint8_t reg, uint16_t& v) {
    Wire.beginTransmission(INA219_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom((int)INA219_ADDR, 2) != 2) return false;
    v = (uint16_t)Wire.read() << 8;
    v |= Wire.read();
    return true;
}

bool loadUsed(float& wh) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;  // nothing saved yet
    uint32_t mWh;
    bool ok = nvs_get_u32(h, NVS_KEY, &mWh) == ESP_OK;
    nvs_close(h);
    if (ok) wh = mWh / 1000.0f;
    return ok;
}

bool saveUsed(float wh) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return false;
    bool ok = nvs_set_u32(h, NVS_KEY, (uint32_t)lroundf(max(wh, 0.0f) * 1000.0f)) == ESP_OK &&
              nvs_commit(h) == ESP_OK;
    nvs_close(h);
    return ok;
}

void publish(const PowerStats& s) {
    portENTER_CRITICAL(&mux);
    stats = s;
    portEXIT_CRITICAL(&mux);
}

// PcbChassis: hand the measured motor supply to the motor driver. A rise is
// taken at once and a drop low-passed, so the duty errs low; an impossible
// reading (or none) makes the motors assume a full battery again.
void feedMotorSupply(float volts, float dt, float& filtered) {
    if (!(volts >= MOTOR_SUPPLY_MIN_V && volts <= MOTOR_SUPPLY_MAX_V)) {
        filtered = NAN;
    } else if (isnan(filtered) || volts > filtered) {
        filtered = volts;
    } else {
        filtered += dt / (MOTOR_SUPPLY_FILTER_MS * 1e-3f + dt) * (volts - filtered);
    }
    motorsSetSupplyVoltage(filtered);
}

void powerTask(void*) {
    bool     configured = false;
    float    supplyV = NAN;     // filtered motor supply (PcbChassis)
    uint32_t nextTryMs = 0;
    bool     lowCandidate = false;
    uint32_t lowSinceMs = 0;
    float    avgW = 0.0f;
    bool     avgReady = false;
    uint32_t lastUs = micros();
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_MS));
        uint32_t nowUs = micros();
        float dt = (nowUs - lastUs) * 1e-6f;
        lastUs = nowUs;

        bool justReset = resetRequested;
        if (justReset) {
            resetRequested = false;
            usedWh = 0.0;
        }

        if (!configured && (int32_t)(millis() - nextTryMs) >= 0) {
            configured = writeReg(REG_CONFIG, INA_CONFIG);
            nextTryMs = millis() + RETRY_MS;
        }

        uint16_t rawShunt = 0, rawBus = 0;
        bool ok = configured && readReg(REG_SHUNT, rawShunt) && readReg(REG_BUS, rawBus);
        if (!ok) {
            configured = false;
            if (INA219_ON_BATTERY) feedMotorSupply(NAN, dt, supplyV);
            PowerStats s = {};
            s.usedWh = (float)usedWh;
            s.usableWh = USABLE_WH;
            s.percent = constrain(1.0f - s.usedWh / USABLE_WH, 0.0f, 1.0f);
            publish(s);
            if (justReset) saveNow = true;
            continue;
        }

        float vShunt = (int16_t)rawShunt * 10e-6f;
        float busV = (rawBus >> 3) * 0.004f;    // measured after the shunt (Vin-)
        float amps = vShunt / INA219_SHUNT_OHMS;
        float inV = busV + vShunt;              // before the shunt (Vin+)
        float inW = inV * amps;
        // Original: Vin+ is the MINI560 output, fed from the battery through
        // the buck. PcbChassis: Vin+ is the pack itself.
        float battW = INA219_ON_BATTERY ? max(inW, 0.0f) : max(inW, 0.0f) / BUCK_EFFICIENCY;

        // PcbChassis: the DRV8833 is fed after the shunt, so its supply is busV.
        if (INA219_ON_BATTERY) feedMotorSupply(busV, dt, supplyV);

        usedWh += battW * dt / 3600.0;

        if (!avgReady) {
            avgW = battW;
            avgReady = true;
        } else {
            avgW += dt / (AVG_TAU_S + dt) * (battW - avgW);
        }

        // Battery nearly empty: the pack (PcbChassis) or the buck output
        // (Original) sags while it is supplying current. (On USB power with
        // the switch off, no current flows through the shunt.)
        if (inV < (INA219_ON_BATTERY ? BATTERY_LOW_V : RAIL_LOW_V) && amps > LOW_MIN_A) {
            if (!lowCandidate) {
                lowCandidate = true;
                lowSinceMs = millis();
            }
        } else {
            lowCandidate = false;
        }

        float leftWh = USABLE_WH - (float)usedWh;
        PowerStats s;
        s.present = true;
        s.busV = busV;
        s.currentA = amps;
        s.powerW = inW;
        s.avgPowerW = avgW;
        s.usedWh = (float)usedWh;
        s.usableWh = USABLE_WH;
        s.percent = constrain(leftWh / USABLE_WH, 0.0f, 1.0f);
        s.timeLeftS = (avgW > 0.05f && leftWh > 0.0f) ? (uint32_t)(leftWh / avgW * 3600.0f) : 0;
        s.low = lowCandidate && millis() - lowSinceMs >= LOW_HOLD_MS;
        publish(s);

        if (justReset) saveNow = true;  // after publishing, so the save sees 0 Wh
    }
}

}  // namespace

void powerBegin() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

    float wh = 0.0f;
    if (loadUsed(wh)) usedWh = wh;
    savedWh = wh;
    lastSaveMs = millis();

    PowerStats s = {};
    s.usedWh = wh;
    s.usableWh = USABLE_WH;
    s.percent = constrain(1.0f - wh / USABLE_WH, 0.0f, 1.0f);
    publish(s);

    xTaskCreatePinnedToCore(powerTask, "power", 3072, nullptr, 1, nullptr, 0);
}

PowerStats powerStats() {
    portENTER_CRITICAL(&mux);
    PowerStats s = stats;
    portEXIT_CRITICAL(&mux);
    return s;
}

void powerBatteryCharged() {
    resetRequested = true;
}

void powerService(bool flashAllowed) {
    if (!flashAllowed) return;
    bool urgent = saveNow;
    if (!urgent && millis() - lastSaveMs < SAVE_EVERY_MS) return;

    float wh = powerStats().usedWh;
    lastSaveMs = millis();
    if (!urgent && fabsf(wh - savedWh) < 0.005f) return;  // nothing worth writing
    if (saveUsed(wh)) savedWh = wh;
    saveNow = false;
}

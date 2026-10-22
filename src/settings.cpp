#include "settings.h"

#include <Arduino.h>
#include <Preferences.h>
#include <nvs.h>

#include "config.h"

namespace {

constexpr char     NVS_NAMESPACE[] = "tuning";
constexpr char     NVS_KEY[] = "t";
constexpr uint16_t RECORD_VERSION = 1;  // bump when Tuning changes shape

struct Record {
    uint16_t version;
    uint32_t defaultsHash;
    Tuning   tuning;
};

// FNV-1a over the default values: changes whenever config.h defaults change.
uint32_t defaultsHash() {
    Tuning d = settingsDefaults();
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&d);
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < sizeof d; i++) h = (h ^ p[i]) * 16777619u;
    return h;
}

}  // namespace

Tuning settingsDefaults() {
    return {BASE_SPEED, MANUAL_MAX_SPEED, KP, KD, MOTOR_TRIM};
}

bool settingsLoad(Tuning& out) {
    // The ESP-IDF API directly: Preferences logs an error when the namespace
    // doesn't exist yet, which is the normal case before the first save.
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;

    Record r;
    size_t len = sizeof r;
    bool ok = nvs_get_blob(h, NVS_KEY, &r, &len) == ESP_OK && len == sizeof r &&
              r.version == RECORD_VERSION &&
              r.defaultsHash == defaultsHash();
    nvs_close(h);

    if (ok) out = r.tuning;
    return ok;
}

bool settingsSave(const Tuning& t) {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) return false;
    Record r = {RECORD_VERSION, defaultsHash(), t};
    bool ok = prefs.putBytes(NVS_KEY, &r, sizeof r) == sizeof r;
    prefs.end();
    return ok;
}

namespace {

constexpr char     CAL_NAMESPACE[] = "sensors";
constexpr char     CAL_KEY[] = "cal";
constexpr uint16_t CAL_VERSION = 1;

struct CalRecord {
    uint16_t  version;
    uint32_t  pinsHash;  // the calibration belongs to these pins, in this order
    SensorCal cal;
};

uint32_t pinsHash() {
    uint32_t h = 2166136261u;
    for (uint8_t i = 0; i < IR_COUNT; i++) h = (h ^ PIN_IR[i]) * 16777619u;
    return (h ^ IR_COUNT) * 16777619u;
}

}  // namespace

bool settingsLoadCal(SensorCal& out) {
    nvs_handle_t h;
    if (nvs_open(CAL_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;  // never calibrated

    CalRecord r;
    size_t len = sizeof r;
    bool ok = nvs_get_blob(h, CAL_KEY, &r, &len) == ESP_OK && len == sizeof r &&
              r.version == CAL_VERSION && r.pinsHash == pinsHash();
    nvs_close(h);

    if (ok) out = r.cal;
    return ok;
}

bool settingsSaveCal(const SensorCal& c) {
    Preferences prefs;
    if (!prefs.begin(CAL_NAMESPACE, false)) return false;
    CalRecord r = {CAL_VERSION, pinsHash(), c};
    bool ok = prefs.putBytes(CAL_KEY, &r, sizeof r) == sizeof r;
    prefs.end();
    return ok;
}

void settingsErase() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) return;
    prefs.clear();
    prefs.end();
}

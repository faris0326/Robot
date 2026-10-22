#include "settings.h"

#include <Arduino.h>
#include <Preferences.h>
#include <nvs.h>

#include "config.h"

namespace {

constexpr char     NVS_NAMESPACE[] = "tuning";
constexpr char     NVS_KEY[] = "t";
constexpr uint16_t RECORD_VERSION = 2;  // bump when Tuning changes shape

struct Record {
    uint16_t version;
    uint32_t defaultsHash;
    Tuning   tuning;
};

// Version 1 had only the first five Tuning fields (no corner brake / spin).
constexpr size_t V1_FIELDS = 5;
struct RecordV1 {
    uint16_t version;
    uint32_t defaultsHash;
    float    fields[V1_FIELDS];
};

// FNV-1a over the first `bytes` of the default values: changes whenever the
// config.h defaults change.
uint32_t defaultsHash(size_t bytes = sizeof(Tuning)) {
    Tuning d = settingsDefaults();
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&d);
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < bytes; i++) h = (h ^ p[i]) * 16777619u;
    return h;
}

}  // namespace

Tuning settingsDefaults() {
    return {BASE_SPEED, MANUAL_MAX_SPEED, KP, KD, MOTOR_TRIM, CORNER_BRAKE_MS, PIVOT_SPEED};
}

bool settingsLoad(Tuning& out) {
    // The ESP-IDF API directly: Preferences logs an error when the namespace
    // doesn't exist yet, which is the normal case before the first save.
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;

    union {
        Record   v2;
        RecordV1 v1;
    } r;
    size_t len = sizeof r;
    bool read = nvs_get_blob(h, NVS_KEY, &r, &len) == ESP_OK;
    nvs_close(h);
    if (!read) return false;

    if (len == sizeof r.v2 && r.v2.version == RECORD_VERSION && r.v2.defaultsHash == defaultsHash()) {
        out = r.v2.tuning;
        return true;
    }
    // Saved before the corner settings existed: keep the five saved fields,
    // the new ones start from config.h.
    static_assert(sizeof(float) * V1_FIELDS == offsetof(Tuning, cornerBrakeMs), "v1 = first five fields");
    if (len == sizeof r.v1 && r.v1.version == 1 && r.v1.defaultsHash == defaultsHash(sizeof r.v1.fields)) {
        out = settingsDefaults();
        memcpy(&out, r.v1.fields, sizeof r.v1.fields);
        return true;
    }
    return false;
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
constexpr uint16_t CAL_VERSION = 2;  // 2: background/line averages (1 was min/max)

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

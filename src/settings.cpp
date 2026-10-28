#include "settings.h"

#include <Arduino.h>
#include <Preferences.h>
#include <nvs.h>

#include "config.h"

namespace {

constexpr char     NVS_NAMESPACE[] = "tuning";
constexpr char     NVS_KEY[] = "t";
constexpr uint16_t RECORD_VERSION = 4;  // bump when Tuning changes shape

struct Record {
    uint16_t version;
    uint32_t defaultsHash;
    Tuning   tuning;
};

// Older records hold only the first Tuning fields: version 1 the first five
// (no corner brake / spin), version 2 the first seven (no calibrate speed),
// version 3 the first eight (no smoothing).
constexpr size_t oldFields(uint16_t version) {
    return version == 1 ? 5 : version == 2 ? 7 : version == 3 ? 8 : 0;
}
static_assert(offsetof(Tuning, cornerBrakeMs) == 5 * sizeof(float), "version 1 = first five fields");
static_assert(offsetof(Tuning, calSpeed) == 7 * sizeof(float), "version 2 = first seven fields");
static_assert(offsetof(Tuning, smoothMs) == 8 * sizeof(float), "version 3 = first eight fields");
struct OldRecord {
    uint16_t version;
    uint32_t defaultsHash;
    float    fields[8];
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
    return {BASE_SPEED, MANUAL_MAX_SPEED, KP, KD, MOTOR_TRIM, CORNER_BRAKE_MS, PIVOT_SPEED, CAL_SPIN_SPEED,
            POSITION_SMOOTH_MS};
}

bool settingsLoad(Tuning& out) {
    // The ESP-IDF API directly: Preferences logs an error when the namespace
    // doesn't exist yet, which is the normal case before the first save.
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;

    union {
        Record    now;
        OldRecord old;
    } r;
    size_t len = sizeof r;
    bool read = nvs_get_blob(h, NVS_KEY, &r, &len) == ESP_OK;
    nvs_close(h);
    if (!read) return false;

    if (len == sizeof r.now && r.now.version == RECORD_VERSION && r.now.defaultsHash == defaultsHash()) {
        out = r.now.tuning;
        return true;
    }
    // Saved by older firmware with fewer settings: keep the saved ones, the
    // newer ones start from config.h.
    size_t bytes = oldFields(r.old.version) * sizeof(float);
    if (bytes && len == offsetof(OldRecord, fields) + bytes && r.old.defaultsHash == defaultsHash(bytes)) {
        out = settingsDefaults();
        memcpy(&out, r.old.fields, bytes);
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
constexpr char     LINE_KEY[] = "white";  // line colour from the phone, u8
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

bool settingsLoadLineWhite(bool& white) {
    nvs_handle_t h;
    if (nvs_open(CAL_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;
    uint8_t v;
    bool ok = nvs_get_u8(h, LINE_KEY, &v) == ESP_OK;
    nvs_close(h);
    if (ok) white = v != 0;
    return ok;
}

bool settingsSaveLineWhite(bool white) {
    Preferences prefs;
    if (!prefs.begin(CAL_NAMESPACE, false)) return false;
    bool ok = prefs.putUChar(LINE_KEY, white ? 1 : 0) == 1;
    prefs.end();
    return ok;
}

void settingsErase() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) return;
    prefs.clear();
    prefs.end();
}

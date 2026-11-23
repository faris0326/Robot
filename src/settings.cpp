#include "settings.h"

#include <Arduino.h>
#include <Preferences.h>
#include <nvs.h>

#include "config.h"

namespace {

constexpr char     NVS_NAMESPACE[] = "tuning";
constexpr char     NVS_KEY[] = "t";
constexpr uint16_t RECORD_VERSION = 6;  // bump when Tuning changes shape (skip 7, see below)

struct Record {
    uint16_t version;
    uint32_t defaultsHash;
    Tuning   tuning;
};

// Older records hold only the first Tuning fields: version 1 the first five
// (no corner brake / spin), version 2 the first seven (no calibrate speed),
// version 3 the first eight (no smoothing), version 4 the first nine (no gap),
// version 5 the first ten (no downhill).
constexpr size_t oldFields(uint16_t version) {
    return version == 1 ? 5 : version == 2 ? 7 : version == 3 ? 8 : version == 4 ? 9 : version == 5 ? 10 : 0;
}

// Version 7 briefly added two long-straight fields after these twelve (later
// removed): read its first twelve. Its hash covered those two defaults too.
constexpr size_t V7_FIELDS = 14;
constexpr float  V7_EXTRA_DEFAULTS[2] = {0.70f, 1000.0f};
static_assert(offsetof(Tuning, cornerBrakeMs) == 5 * sizeof(float), "version 1 = first five fields");
static_assert(offsetof(Tuning, calSpeed) == 7 * sizeof(float), "version 2 = first seven fields");
static_assert(offsetof(Tuning, smoothMs) == 8 * sizeof(float), "version 3 = first eight fields");
static_assert(offsetof(Tuning, gapMs) == 9 * sizeof(float), "version 4 = first nine fields");
static_assert(offsetof(Tuning, downhillScale) == 10 * sizeof(float), "version 5 = first ten fields");
static_assert(sizeof(Tuning) == 12 * sizeof(float), "version 7 = these twelve fields + two");
struct OldRecord {
    uint16_t version;
    uint32_t defaultsHash;
    float    fields[V7_FIELDS];
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
            POSITION_SMOOTH_MS, GAP_STRAIGHT_MS, DOWNHILL_SCALE, DOWNHILL_MS};
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
    if (r.old.version == 7 && len == offsetof(OldRecord, fields) + V7_FIELDS * sizeof(float)) {
        float d[V7_FIELDS];
        Tuning defaults = settingsDefaults();
        memcpy(d, &defaults, sizeof defaults);
        memcpy(d + sizeof defaults / sizeof(float), V7_EXTRA_DEFAULTS, sizeof V7_EXTRA_DEFAULTS);
        uint32_t hash = 2166136261u;
        const uint8_t* p = reinterpret_cast<const uint8_t*>(d);
        for (size_t i = 0; i < sizeof d; i++) hash = (hash ^ p[i]) * 16777619u;
        if (r.old.defaultsHash != hash) return false;
        memcpy(&out, r.old.fields, sizeof out);
        return true;
    }

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

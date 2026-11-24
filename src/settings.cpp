#include "settings.h"

#include <Arduino.h>
#include <Preferences.h>
#include <nvs.h>

#include "config.h"

namespace {

constexpr char     NVS_NAMESPACE[] = "tuning";
constexpr char     NVS_KEY[] = "t";
constexpr uint16_t RECORD_VERSION = 9;  // bump when Tuning changes shape

struct Record {
    uint16_t version;
    uint32_t defaultsHash;
    Tuning   tuning;
};

// Every Tuning field is a float. The first COMMON (auto speed .. gap ms) are
// the same in every saved version; speed control (3) follows them since v9.
constexpr size_t COMMON = 10;
constexpr size_t CRUISE_FIELDS = 3;
static_assert(offsetof(Tuning, cruiseMms) == COMMON * sizeof(float), "speed control right after the common fields");
static_assert(sizeof(Tuning) == (COMMON + CRUISE_FIELDS) * sizeof(float), "Tuning is floats only");

// Older versions, read so saved sliders survive updates:
//   1..5   the first 5 / 7 / 8 / 9 / 10 common fields
//   6      common + downhill speed, downhill ms (removed in v9: speed control replaced it)
//   7      common + downhill + two long-straight fields (removed)
//   8      common + downhill + speed control
// Each record's hash covered the config.h defaults of its own fields.
constexpr float  OLD_DOWNHILL[2] = {0.60f, 1000.0f};
constexpr float  OLD_STRAIGHT[2] = {0.70f, 1000.0f};
constexpr size_t MAX_OLD_FIELDS = 15;
struct OldRecord {
    uint16_t version;
    uint32_t defaultsHash;
    float    fields[MAX_OLD_FIELDS];
};

uint32_t fnv(const void* data, size_t bytes) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(data);
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < bytes; i++) h = (h ^ p[i]) * 16777619u;
    return h;
}

// An old version's field count, its defaults (for the hash) and where its
// speed-control fields sit (-1 = none). False for an unknown version.
bool oldLayout(uint16_t version, float* defaults, size_t& count, int& cruiseAt) {
    static const size_t COUNT[] = {0, 5, 7, 8, 9, 10, 12, 14, 15};
    if (version < 1 || version > 8) return false;
    count = COUNT[version];
    cruiseAt = version == 8 ? COMMON + 2 : -1;
    Tuning d = settingsDefaults();
    const float* now = reinterpret_cast<const float*>(&d);
    memcpy(defaults, now, min(count, COMMON) * sizeof(float));
    if (version >= 6) memcpy(defaults + COMMON, OLD_DOWNHILL, sizeof OLD_DOWNHILL);
    if (version == 7) memcpy(defaults + COMMON + 2, OLD_STRAIGHT, sizeof OLD_STRAIGHT);
    if (version == 8) memcpy(defaults + COMMON + 2, now + COMMON, CRUISE_FIELDS * sizeof(float));
    return true;
}

// FNV-1a over the default values: changes whenever the config.h defaults change.
uint32_t defaultsHash() {
    Tuning d = settingsDefaults();
    return fnv(&d, sizeof d);
}

}  // namespace

Tuning settingsDefaults() {
    return {BASE_SPEED, MANUAL_MAX_SPEED, KP, KD, MOTOR_TRIM, CORNER_BRAKE_MS, PIVOT_SPEED, CAL_SPIN_SPEED,
            POSITION_SMOOTH_MS, GAP_STRAIGHT_MS, CRUISE_MMS, CRUISE_KP, CRUISE_KI};
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
    // Saved by older firmware: keep the fields this version still has, the
    // rest start from config.h.
    float defaults[MAX_OLD_FIELDS];
    size_t count;
    int cruiseAt;
    if (!oldLayout(r.old.version, defaults, count, cruiseAt)) return false;
    if (len != offsetof(OldRecord, fields) + count * sizeof(float)) return false;
    if (r.old.defaultsHash != fnv(defaults, count * sizeof(float))) return false;
    out = settingsDefaults();
    float* to = reinterpret_cast<float*>(&out);
    memcpy(to, r.old.fields, min(count, COMMON) * sizeof(float));
    if (cruiseAt >= 0) memcpy(to + COMMON, r.old.fields + cruiseAt, CRUISE_FIELDS * sizeof(float));
    return true;
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

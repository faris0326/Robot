#include "telemetry.h"

#include <Arduino.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>

#include "config.h"
#include "control.h"
#include "motors.h"
#include "settings.h"
#include "shared.h"
#include "power.h"
#include "sysmon.h"

// WPA2 rejects other lengths and softAP() then fails without any sign.
static_assert(sizeof(WIFI_PASS) == 1 || (sizeof(WIFI_PASS) - 1 >= 8 && sizeof(WIFI_PASS) - 1 <= 63),
              "WIFI_PASS must be 8..63 characters, or empty for an open network");
static_assert(sizeof(WIFI_SSID) - 1 >= 1 && sizeof(WIFI_SSID) - 1 <= 32, "WIFI_SSID must be 1..32 characters");

// web/index.html, embedded by board_build.embed_txtfiles (null-terminated)
extern const char INDEX_HTML[] asm("_binary_web_index_html_start");

namespace {

constexpr uint8_t  MSG_SAMPLE = 1;
constexpr uint8_t  MSG_LAP = 2;
constexpr uint8_t  CMD_MODE = 0x10;
constexpr uint8_t  CMD_DRIVE = 0x11;
constexpr uint8_t  CMD_TUNING = 0x12;
constexpr uint8_t  CMD_TUNING_RESET = 0x13;
constexpr uint8_t  CMD_BATTERY_CHARGED = 0x14;
constexpr uint8_t  CMD_LINE_COLOUR = 0x15;
constexpr uint8_t  CMD_VOLT_LIMIT = 0x16;
constexpr uint8_t  FRAME_VERSION = 2;  // sample frame; v2 added the sensor levels
constexpr uint8_t  LAP_VERSION = 1;
constexpr size_t   SAMPLE_FRAME_SIZE = 37 + IR_COUNT;
constexpr size_t   LAP_FRAME_SIZE = 16;
constexpr uint32_t FRAME_INTERVAL_MS = 1000 / TELEMETRY_HZ;
constexpr size_t   LAP_HISTORY = 200;

WebServer        http(80);
WebSocketsServer ws(WS_PORT);

// Every lap event so far, replayed to each phone that connects.
LapEvent lapHistory[LAP_HISTORY];
size_t   lapHistoryCount = 0;

// Saving tuning to flash
enum class SaveState : uint8_t { Saved, Pending, Waiting, Failed };
SaveState saveState = SaveState::Saved;
uint32_t  tuningChangedMs = 0;
Tuning    flashTuning = {};   // what flash holds
bool      onFlash = false;    // false: nothing saved, running on config.h values

// Sensor calibration
const char* calStatus = "default";  // "saved" or "default"
uint32_t    calSeenSeq = 0;
bool        lineSavePending = false;  // line colour changed, not in flash yet

void put8(uint8_t* p, uint8_t v) { p[0] = v; }
void put16(uint8_t* p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
void put32(uint8_t* p, uint32_t v) { put16(p, v); put16(p + 2, v >> 16); }

int16_t scaled(float v) {
    return (int16_t)lroundf(constrain(v, -1.0f, 1.0f) * 10000.0f);
}

size_t encodeSample(const TelemetrySample& s, const PowerStats& pw, uint8_t* f) {
    memset(f, 0, SAMPLE_FRAME_SIZE);
    put8(f + 0, MSG_SAMPLE);
    put8(f + 1, FRAME_VERSION);
    put16(f + 2, (uint16_t)s.sensorBits);
    put32(f + 4, s.timeMs);
    put16(f + 8, scaled(s.position));
    put16(f + 10, scaled(s.dutyLeft));
    put16(f + 12, scaled(s.dutyRight));
    put8(f + 14, s.lineState);
    put8(f + 15, (s.running ? 1 : 0) | ((uint8_t)s.mode & 7) << 1);
    put16(f + 16, s.lap);
    put32(f + 20, s.lastLapMs);
    put16(f + 24, s.loopMaxUs);
    put16(f + 26, s.periodMaxUs);
    // 28..31 reserved for encoder speed, left as zero
    if (pw.present) {
        put16(f + 32, (uint16_t)lroundf(constrain(pw.busV, 0.0f, 65.0f) * 1000.0f));
        put16(f + 34, (int16_t)lroundf(constrain(pw.currentA, -32.0f, 32.0f) * 1000.0f));
    }
    put8(f + 36, IR_COUNT);
    memcpy(f + 37, s.levels, IR_COUNT);
    return SAMPLE_FRAME_SIZE;
}

size_t encodeLap(const LapEvent& ev, bool replay, uint8_t* f) {
    memset(f, 0, LAP_FRAME_SIZE);
    put8(f + 0, MSG_LAP);
    put8(f + 1, LAP_VERSION);
    put16(f + 2, ev.lap);
    put32(f + 4, ev.lapMs);
    put32(f + 8, ev.startMs);
    put8(f + 12, replay ? 1 : 0);
    return LAP_FRAME_SIZE;
}

void forwardLapEvents() {
    LapEvent ev;
    uint8_t  f[LAP_FRAME_SIZE];
    while (sharedPopLap(ev)) {
        if (lapHistoryCount < LAP_HISTORY) lapHistory[lapHistoryCount++] = ev;
        size_t len = encodeLap(ev, false, f);
        ws.broadcastBIN(f, len);
    }
}

void replayLaps(uint8_t client) {
    uint8_t f[LAP_FRAME_SIZE];
    for (size_t i = 0; i < lapHistoryCount; i++) {
        size_t len = encodeLap(lapHistory[i], true, f);
        ws.sendBIN(client, f, len);
    }
}

const char* saveStatus() {
    switch (saveState) {
        case SaveState::Pending: return "pending";
        case SaveState::Waiting: return "waiting";
        case SaveState::Failed:  return "error";
        default:                 return onFlash ? "saved" : "defaults";
    }
}

// The tuning fields as JSON members (no braces), shared by hello and tuning.
void formatTuning(char* out, size_t n) {
    Tuning t = sharedTuning();
    snprintf(out, n,
             "\"autoSpeed\":%.3f,\"manualMax\":%.3f,\"kp\":%.3f,\"kd\":%.4f,\"trim\":%.3f,"
             "\"brakeMs\":%.0f,\"pivot\":%.3f,\"calSpeed\":%.3f,\"smoothMs\":%.0f,\"save\":\"%s\"",
             t.autoSpeed, t.manualMax, t.kp, t.kd, t.trim, t.cornerBrakeMs, t.pivotSpeed, t.calSpeed,
             t.smoothMs, saveStatus());
}

void sendHello(uint8_t client) {
    char tune[200];
    formatTuning(tune, sizeof tune);
    char msg[448];
    snprintf(msg, sizeof msg,
             "{\"type\":\"hello\",\"frame\":%u,\"ir\":%u,\"hz\":%lu,\"ctrlUs\":%lu,"
             "\"motorV\":%.2f,\"vLimit\":%.2f,\"vMax\":%.2f,\"inaBattery\":%s,\"laps\":%s,"
             "\"cal\":\"%s\",\"white\":%s,%s}",
             FRAME_VERSION, IR_COUNT, (unsigned long)TELEMETRY_HZ,
             (unsigned long)CONTROL_PERIOD_US, MOTOR_RATED_V, motorsVoltageLimit(), MOTOR_UNLOCK_MAX_V,
             INA219_ON_BATTERY ? "true" : "false", LAP_TIMING_ENABLED ? "true" : "false", calStatus,
             sensorsLineWhite() ? "true" : "false", tune);
    ws.sendTXT(client, msg);
}

// A calibration finished on the control task: save it and tell the phones.
// The robot is in Stop by now, so the flash write can't disturb driving.
void serviceCalibration() {
    bool   ok;
    int8_t bad;
    uint32_t seq = sharedCalibrationResult(ok, bad);
    if (seq == calSeenSeq) return;
    calSeenSeq = seq;

    bool saved = ok && settingsSaveCal(sensorsCalibration());
    if (saved) calStatus = "saved";

    SensorCal c = sensorsCalibration();
    char msg[384];
    int n = snprintf(msg, sizeof msg, "{\"type\":\"cal\",\"ok\":%s,\"bad\":%d,\"saved\":%s,\"white\":%s,\"bg\":[",
                     ok ? "true" : "false", bad, saved ? "true" : "false", calLineIsWhite(c) ? "true" : "false");
    for (uint8_t i = 0; i < IR_COUNT; i++) n += snprintf(msg + n, sizeof msg - n, "%s%u", i ? "," : "", c.background[i]);
    n += snprintf(msg + n, sizeof msg - n, "],\"line\":[");
    for (uint8_t i = 0; i < IR_COUNT; i++) n += snprintf(msg + n, sizeof msg - n, "%s%u", i ? "," : "", c.line[i]);
    snprintf(msg + n, sizeof msg - n, "]}");
    ws.broadcastTXT(msg);
}

// Tell every phone the current tuning, so all sliders agree.
void broadcastTuning() {
    char tune[200];
    formatTuning(tune, sizeof tune);
    char msg[240];
    snprintf(msg, sizeof msg, "{\"type\":\"tuning\",%s}", tune);
    ws.broadcastTXT(msg);
}

void broadcastStats() {
    SysStats st;
    sysmonSample(st);
    PowerStats pw = powerStats();
    char msg[512];
    snprintf(msg, sizeof msg,
             "{\"type\":\"stats\",\"cpu0\":%.3f,\"cpu1\":%.3f,\"ctrlLoad\":%.4f,"
             "\"heap\":%lu,\"heapMin\":%lu,\"psram\":%lu,\"temp\":%.1f,"
             "\"rssi\":%d,\"clients\":%u,\"uptime\":%lu,\"mhz\":%u,"
             "\"ina\":%s,\"battPct\":%.3f,\"battLeftS\":%lu,\"battUsedWh\":%.3f,"
             "\"battUsableWh\":%.2f,\"avgW\":%.3f,\"battLow\":%s}",
             st.cpuLoad[0], st.cpuLoad[1], st.controlLoad,
             (unsigned long)st.heapFree, (unsigned long)st.heapMin, (unsigned long)st.psramFree,
             st.tempC, st.rssi, st.clients, (unsigned long)st.uptimeS, st.cpuMhz,
             pw.present ? "true" : "false", pw.percent, (unsigned long)pw.timeLeftS, pw.usedWh,
             pw.usableWh, pw.avgPowerW, pw.low ? "true" : "false");
    ws.broadcastTXT(msg);
}

int16_t get16(const uint8_t* p) { return (int16_t)(p[0] | p[1] << 8); }

float getF32(const uint8_t* p) {  // little-endian IEEE 754, same as the ESP32
    float v;
    memcpy(&v, p, sizeof v);
    return v;
}

void onCommand(const uint8_t* p, size_t len) {
    if (len >= 2 && p[0] == CMD_MODE && p[1] <= (uint8_t)DriveMode::Hold) {
        sharedSetMode((DriveMode)p[1]);
    } else if (len >= 5 && p[0] == CMD_DRIVE) {
        sharedSetDrive(get16(p + 1) / 10000.0f, get16(p + 3) / 10000.0f);
    } else if (len >= 21 && p[0] == CMD_TUNING) {
        Tuning t = sharedTuning();
        t = {getF32(p + 1), getF32(p + 5), getF32(p + 9), getF32(p + 13), getF32(p + 17),
             t.cornerBrakeMs, t.pivotSpeed, t.calSpeed, t.smoothMs};
        if (len >= 29) {
            t.cornerBrakeMs = getF32(p + 21);
            t.pivotSpeed = getF32(p + 25);
        }
        if (len >= 33) t.calSpeed = getF32(p + 29);
        if (len >= 37) t.smoothMs = getF32(p + 33);
        sharedSetTuning(t);
        saveState = SaveState::Pending;
        tuningChangedMs = millis();
        broadcastTuning();
    } else if (len >= 1 && p[0] == CMD_TUNING_RESET) {
        settingsErase();
        sharedSetTuning(settingsDefaults());
        onFlash = false;
        saveState = SaveState::Saved;
        broadcastTuning();
    } else if (len >= 1 && p[0] == CMD_BATTERY_CHARGED) {
        powerBatteryCharged();
    } else if (len >= 2 && p[0] == CMD_LINE_COLOUR) {
        sensorsSetLineWhite(p[1] != 0);
        lineSavePending = true;
        char msg[48];
        snprintf(msg, sizeof msg, "{\"type\":\"line\",\"white\":%s}", p[1] ? "true" : "false");
        ws.broadcastTXT(msg);
    } else if (len >= 5 && p[0] == CMD_VOLT_LIMIT) {
        motorsSetVoltageLimit(getF32(p + 1));
        float v = motorsVoltageLimit();
        Serial.printf("Motor voltage limit: %.1f V%s\n", v, v > MOTOR_RATED_V + 0.01f ? " (UNLOCKED, overdrive)" : "");
        char msg[48];
        snprintf(msg, sizeof msg, "{\"type\":\"vlimit\",\"v\":%.2f}", v);
        ws.broadcastTXT(msg);
    }
}

// Flash writes briefly pause both cores, so none happen while Auto (or the
// Center test, which steers the same way) drives.
bool autoDriving() {
    TelemetrySample s = sharedLatest();
    return (s.mode == DriveMode::Auto || s.mode == DriveMode::Hold) && s.running &&
           (s.dutyLeft != 0.0f || s.dutyRight != 0.0f);
}

// Write changed tuning to flash once the sliders have settled, but not while
// Auto is driving.
void serviceSave() {
    if (saveState == SaveState::Saved || saveState == SaveState::Failed) return;
    if (millis() - tuningChangedMs < TUNING_SAVE_DELAY_MS) return;

    if (autoDriving()) {
        if (saveState != SaveState::Waiting) {
            saveState = SaveState::Waiting;
            broadcastTuning();
        }
        return;
    }

    Tuning t = sharedTuning();
    bool ok = true;
    if (!onFlash || memcmp(&t, &flashTuning, sizeof t) != 0) {  // skip writes that change nothing
        ok = settingsSave(t);
        if (ok) {
            flashTuning = t;
            onFlash = true;
        }
    }
    saveState = ok ? SaveState::Saved : SaveState::Failed;
    broadcastTuning();
}

void onWsEvent(uint8_t client, WStype_t type, uint8_t* payload, size_t len) {
    if (type == WStype_CONNECTED) {
        sendHello(client);
        replayLaps(client);
    } else if (type == WStype_BIN) {
        onCommand(payload, len);
    }
}

void netTask(void*) {
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(WIFI_SSID, WIFI_PASS[0] ? WIFI_PASS : nullptr, WIFI_CHANNEL, 0, WIFI_MAX_CLIENTS)) {
        Serial.println("Telemetry: Wi-Fi access point FAILED to start");
    }
    WiFi.setSleep(false);

    // no-store: phones would otherwise keep showing an old page after an upload.
    http.on("/", [] {
        http.sendHeader("Cache-Control", "no-store");
        http.send_P(200, "text/html", INDEX_HTML);
    });
    http.onNotFound([] { http.send(404, "text/plain", "not found"); });
    http.begin();

    ws.begin();
    ws.onEvent(onWsEvent);

    Serial.printf("Telemetry: join Wi-Fi \"%s\", open http://%s\n",
                  WIFI_SSID, WiFi.softAPIP().toString().c_str());

    sysmonBegin();
    onFlash = settingsLoad(flashTuning);
    SensorCal savedCal;
    calStatus = settingsLoadCal(savedCal) ? "saved" : "default";
    bool calOk;
    int8_t calBad;
    calSeenSeq = sharedCalibrationResult(calOk, calBad);

    uint8_t  frame[SAMPLE_FRAME_SIZE];
    uint32_t lastSeq = 0;
    uint32_t nextFrameMs = millis();
    uint32_t nextStatsMs = millis() + 1000;

    for (;;) {
        http.handleClient();
        ws.loop();
        forwardLapEvents();
        serviceSave();
        serviceCalibration();
        if (lineSavePending && !autoDriving()) {
            settingsSaveLineWhite(sensorsLineWhite());
            lineSavePending = false;
        }
        powerService(!autoDriving());

        if ((int32_t)(millis() - nextStatsMs) >= 0) {
            nextStatsMs += 1000;
            if (ws.connectedClients() > 0) broadcastStats();
        }

        uint32_t now = millis();
        if ((int32_t)(now - nextFrameMs) >= 0) {
            nextFrameMs += FRAME_INTERVAL_MS;
            if ((int32_t)(now - nextFrameMs) > (int32_t)FRAME_INTERVAL_MS) nextFrameMs = now + FRAME_INTERVAL_MS;

            TelemetrySample s = sharedLatest();
            if (s.seq != lastSeq && ws.connectedClients() > 0) {
                lastSeq = s.seq;
                size_t len = encodeSample(s, powerStats(), frame);
                ws.broadcastBIN(frame, len);
            }
        }

        vTaskDelay(1);
    }
}

}  // namespace

void telemetryStart() {
    xTaskCreatePinnedToCore(netTask, "net", 8192, nullptr, 2, nullptr, 0);
}

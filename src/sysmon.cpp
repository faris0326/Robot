#include "sysmon.h"

#include <Arduino.h>
#include <esp_freertos_hooks.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <xtensa/core-macros.h>

#include "control_task.h"

namespace {

// Gaps between idle-hook calls shorter than this count as idle.
constexpr uint32_t IDLE_GAP_US = 50;

uint32_t          idleGapCycles = 0;
volatile uint32_t idleCycles[2] = {};  // wraps; only deltas are used
uint32_t          lastCcount[2] = {};

// Previous sample, for per-second deltas
int64_t  lastSampleUs = 0;
uint32_t lastIdle[2] = {};
uint32_t lastBusyUs = 0;

inline void trackIdle(int cpu) {
    uint32_t now = XTHAL_GET_CCOUNT();  // per-core cycle counter
    uint32_t gap = now - lastCcount[cpu];
    lastCcount[cpu] = now;
    if (gap < idleGapCycles) idleCycles[cpu] += gap;
}

// Returning false keeps the idle task looping (no wait-for-interrupt), so
// the hook runs continuously while the core is idle.
bool idleHook0() { trackIdle(0); return false; }
bool idleHook1() { trackIdle(1); return false; }

}  // namespace

void sysmonBegin() {
    idleGapCycles = IDLE_GAP_US * getCpuFrequencyMhz();
    esp_register_freertos_idle_hook_for_cpu(idleHook0, 0);
    esp_register_freertos_idle_hook_for_cpu(idleHook1, 1);
    lastSampleUs = esp_timer_get_time();
}

void sysmonSample(SysStats& s) {
    int64_t  nowUs = esp_timer_get_time();
    float    windowUs = (float)(nowUs - lastSampleUs);
    uint16_t mhz = getCpuFrequencyMhz();

    for (int cpu = 0; cpu < 2; cpu++) {
        uint32_t idle = idleCycles[cpu];
        float idleUs = (float)(idle - lastIdle[cpu]) / mhz;
        lastIdle[cpu] = idle;
        s.cpuLoad[cpu] = windowUs > 0 ? constrain(1.0f - idleUs / windowUs, 0.0f, 1.0f) : 0.0f;
    }

    uint32_t busy = controlBusyUsTotal();
    s.controlLoad = windowUs > 0 ? constrain((busy - lastBusyUs) / windowUs, 0.0f, 1.0f) : 0.0f;
    lastBusyUs = busy;
    lastSampleUs = nowUs;

    s.heapFree = ESP.getFreeHeap();
    s.heapMin = ESP.getMinFreeHeap();
    s.psramFree = ESP.getFreePsram();
    s.tempC = temperatureRead();
    s.uptimeS = (uint32_t)(nowUs / 1000000);
    s.cpuMhz = mhz;

    s.rssi = 0;
    s.clients = 0;
    wifi_sta_list_t list;
    if (esp_wifi_ap_get_sta_list(&list) == ESP_OK) {
        s.clients = list.num;
        for (int i = 0; i < list.num; i++) {
            if (s.rssi == 0 || list.sta[i].rssi > s.rssi) s.rssi = list.sta[i].rssi;
        }
    }
}

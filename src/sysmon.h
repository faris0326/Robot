#pragma once
// ESP32 performance stats for the phone page, sampled once a second by the
// telemetry task.
//
// CPU load per core is measured with FreeRTOS idle hooks (run-time stats are
// disabled in the Arduino core): the hook keeps the idle task spinning and
// adds up the time between its calls. Short gaps are idle time; long gaps
// mean another task or interrupt had the core. Load = 1 - idle share.

#include <stdint.h>

struct SysStats {
    float    cpuLoad[2];    // 0..1 per core
    float    controlLoad;   // control loop busy time / wall time, 0..1
    uint32_t heapFree;      // internal RAM, bytes
    uint32_t heapMin;       // lowest internal free heap since boot
    uint32_t psramFree;     // bytes, 0 if no PSRAM
    float    tempC;         // chip temperature
    int8_t   rssi;          // strongest connected phone, dBm (0 if none)
    uint8_t  clients;       // phones joined to the access point
    uint32_t uptimeS;
    uint16_t cpuMhz;
};

void sysmonBegin();               // registers the idle hooks; call once
void sysmonSample(SysStats& out); // call about once a second

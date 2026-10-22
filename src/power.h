#pragma once
// Power monitor and battery estimate, from the INA219 on the 5 V rail.
//
// A task on core 0 reads the INA219 every 20 ms and integrates the energy
// drawn from the 5 V rail. Divided by the MINI560 efficiency, that is the
// energy taken from the battery. Battery % = 1 - used / usable capacity, so
// it is only right if the phone's "Battery charged" button is pressed after
// each full charge. Energy used is kept in flash across power-offs.

#include <stdint.h>

struct PowerStats {
    bool     present;      // INA219 answered on I2C
    float    railV;        // 5 V rail at the board (after the shunt)
    float    currentA;     // total robot current
    float    powerW;       // power drawn from the 5 V supply
    float    avgPowerW;    // battery-side power, ~30 s average
    float    usedWh;       // battery energy used since the last "charged" reset
    float    usableWh;     // BATTERY_MAH x BATTERY_NOMINAL_V x BATTERY_USABLE
    float    percent;      // 0..1
    uint32_t timeLeftS;    // at avgPowerW; 0 if unknown
    bool     low;          // MINI560 output sagging: battery nearly empty
};

void       powerBegin();          // I2C, INA219 setup, loads the saved energy, starts the task
PowerStats powerStats();
void       powerBatteryCharged(); // reset to 100 %; saved by powerService()

// Writes the energy used to flash about once a minute, and right after a
// reset. Call from the telemetry task; pass false while Auto is driving
// (a flash write pauses both cores).
void powerService(bool flashAllowed);

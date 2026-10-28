#pragma once
// Power monitor and battery estimate, from the INA219 (see BOARD in config.h).
//
// A task on core 0 reads the INA219 every 20 ms and integrates the energy
// taken from the battery: on the Original board the 5 V rail's energy
// divided by the MINI560 efficiency, on PcbChassis the pack's own energy.
// Battery % = 1 - used / usable capacity, so it is only right if the phone's
// "Battery charged" button is pressed after each full charge. Energy used is
// kept in flash across power-offs.
//
// On PcbChassis the task also hands the measured motor supply (the INA219
// bus voltage, which feeds the DRV8833) to motorsSetSupplyVoltage().

#include <stdint.h>

struct PowerStats {
    bool     present;      // INA219 answered on I2C
    float    busV;         // after the shunt: 5 V rail (Original) or battery (PcbChassis)
    float    currentA;     // current through the shunt
    float    powerW;       // power through the shunt
    float    avgPowerW;    // battery-side power, ~30 s average
    float    usedWh;       // battery energy used since the last "charged" reset
    float    usableWh;     // BATTERY_MAH x BATTERY_NOMINAL_V x BATTERY_USABLE
    float    percent;      // 0..1
    uint32_t timeLeftS;    // at avgPowerW; 0 if unknown
    bool     low;          // battery nearly empty (rail sagging, or pack below BATTERY_LOW_V)
};

void       powerBegin();          // I2C, INA219 setup, loads the saved energy, starts the task
PowerStats powerStats();
void       powerBatteryCharged(); // reset to 100 %; saved by powerService()

// Writes the energy used to flash about once a minute, and right after a
// reset. Call from the telemetry task; pass false while Auto is driving
// (a flash write pauses both cores).
void powerService(bool flashAllowed);

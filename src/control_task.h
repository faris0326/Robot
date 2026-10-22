#pragma once
// Fixed-rate control loop pinned to core 1: sensors -> control -> motors,
// then publishes a telemetry sample. Never touches the network.

#include <stdint.h>

void controlTaskStart();

// Total time spent inside the control loop body since boot, in us. Wraps;
// read deltas only.
uint32_t controlBusyUsTotal();

#pragma once

#include <stdint.h>

// How often loop() runs, in seconds.
static constexpr uint32_t LOOP_INTERVAL_S = 1;

// readFrame() is non-blocking; the loop cadence above paces CAN draining.
static constexpr uint32_t CAN_READ_TIMEOUT_MS = 0;

// Debug logging: GPS status (sentence counts, fix validity, wiring hints)
// and invalid RMC sentences to serial. Set false to silence GPS diagnostics.
static constexpr bool GPS_DEBUG = true;

// Debug logging: CAN/OBD-II errors (transmit failures, request timeouts,
// negative ECU responses). Set false to silence CAN diagnostics.
static constexpr bool CAN_DEBUG = true;

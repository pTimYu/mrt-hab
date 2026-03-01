#pragma once

#include "../utils/data_extractor.h"

// ── Flight State Machine ──────────────────────────────────────────────────────
//
// States:
//   STNDBY  — on the ground, waiting for takeoff. Ground altitude is
//             continuously updated. Takeoff detection starts only after
//             STANDBY_SETTLE_MS has elapsed so sensor readings stabilise.
//
//   ASCENT  — airborne and climbing. Data is logged every loop.
//             Flight termination fires if runtime > MAX_FLIGHT_TIME OR
//             altitude above ground > MAX_FLIGHT_HEIGHT.
//             Transitions to DESCNT when vertical_velocity < 0.01 m/s.
//
//   DESCNT  — descending under parachute. Data is logged every loop.
//             Transitions to LANDED when |vertical_velocity| < 0.1 m/s
//             for LANDING_THRESHOLD consecutive loops.
//
//   LANDED  — on the ground. Plays "LAND" in Morse every 60 s.
//             SD file is closed once on entry.
//
// Ownership:
//   fc_update() writes data_set.time (effective flight time ms since takeoff).
//   All other DataSet fields are owned by data_extractor / kalman_update.
// ─────────────────────────────────────────────────────────────────────────────

enum FlightStatus {
    STNDBY,
    ASCENT,
    DESCNT,
    LANDED
};

// Reset internal state. Call once in setup() after all sensors are ready.
void fc_init();

// Run one state-machine cycle.
// Call every loop AFTER data_extractor() and kalman_update() have run.
// Writes data_set.time. Calls data_saver_write() internally during flight.
void fc_update(DataSet& data_set);

// Returns the current flight state (read-only, for serial debug).
FlightStatus fc_get_status();

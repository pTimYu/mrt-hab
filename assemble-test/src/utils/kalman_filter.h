#pragma once

// ── Kalman Filter — Altitude + Vertical Velocity ─────────────────────────────
//
// State vector:  x = [ altitude, vertical_velocity ]
//
// Fuses:
//   - Barometer altitude measurement  (slow, noisy, absolute)
//   - IMU z-axis acceleration         (fast, drifts, relative)
//
// Sensor failure handling:
//   - If baro_valid == false, the measurement update is skipped and the filter
//     runs in predict-only mode using IMU acceleration alone.
//   - If imu_valid == false, accel_z = 0 is assumed (constant velocity model).
//
// Usage:
//   kalman_init();
//   kalman_update(data.bmp_altitude, data.bmp, data.az, data.imu, dt);
//   data.altitude          = kalman_get_altitude();
//   data.vertical_velocity = kalman_get_velocity();
// ─────────────────────────────────────────────────────────────────────────────

// Reset all internal state. Call once in setup() after sensors are ready.
void kalman_init();

// Run one predict-update cycle.
//   baro_altitude : raw altitude from BMP sensor (m, MSL)
//   baro_valid    : pass data.bmp  — skips measurement update if false
//   accel_z       : vertical acceleration (m/s²), gravity-compensated
//   imu_valid     : pass data.imu  — uses 0 acceleration if false
//   dt            : elapsed time since last call (seconds)
void kalman_update(float baro_altitude, bool baro_valid,
                   float accel_z,       bool imu_valid,
                   float dt);

float kalman_get_altitude();   // metres (MSL)
float kalman_get_velocity();   // m/s  (positive = ascending)

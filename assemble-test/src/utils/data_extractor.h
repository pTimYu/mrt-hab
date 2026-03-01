#pragma once

// ── DataSet ───────────────────────────────────────────────────────────────────
//
// Central data container passed between all modules each loop.
//
// OWNERSHIP RULES — who writes what:
//   data_extractor()   → all raw sensor fields (voltage, BMP, IMU, GPS, radio)
//   kalman_update()    → altitude, vertical_velocity        (filtered outputs)
//   flight_controller  → time  (effective flight time in ms)
//
// data_saver_write() reads the whole struct and logs it — it never writes.
// ─────────────────────────────────────────────────────────────────────────────

struct DataSet {

    // ── General ───────────────────────────────────────────────────────────
    int      time;          // Effective flight time (ms since takeoff)
    float    voltage;       // Battery voltage (V)

    // ── BMP390 ────────────────────────────────────────────────────────────
    bool  bmp;              // true = read succeeded this cycle
    float temperature;      // °C
    float pressure;         // hPa
    float bmp_altitude;     // Raw barometer altitude (m, MSL)

    // ── IMU (LSM6DSM) ─────────────────────────────────────────────────────
    bool  imu;              // true = read succeeded this cycle
    float ax, ay, az;       // Acceleration (m/s²)
    float gx, gy, gz;       // Angular rate (°/s)

    // ── GPS ───────────────────────────────────────────────────────────────
    bool  gps;              // False if at least one data wrong
    int   time_now;         // Current time HHMMSS from GNSS (6 digits)
    float latitude;         // °
    float longitude;        // °
    float gps_altitude;     // m (MSL)
    float speed_gps;        // m/s
    int   heading_gps;      // degrees (0–359)

    // ── Radio ─────────────────────────────────────────────────────────────
    float RSSI;             // dBm
    float Gain;             // dB

    // ── Kalman Filter Outputs ─────────────────────────────────────────────
    // Populated by kalman_update() in the main loop, NOT by data_extractor.
    float altitude;           // Filtered altitude (m, MSL)
    float vertical_velocity;  // Filtered vertical velocity (m/s, + = ascending)

};

// Reads all raw sensors and populates the raw fields of data.
// Does NOT touch altitude, vertical_velocity, or time.
void data_extractor(DataSet& data);

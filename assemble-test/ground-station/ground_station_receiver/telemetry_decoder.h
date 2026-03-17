#pragma once

#include <Arduino.h>

// ── Decoded telemetry fields ─────────────────────────────────────────────────
struct TelemetryData {
    int32_t time_hhmmss;        // HHMMSS from GNSS
    float   latitude;           // degrees (signed)
    float   longitude;          // degrees (signed)
    float   speed_gps;          // m/s
    int     heading_gps;        // degrees (0–359)
    float   altitude;           // m (Kalman-filtered)
    float   voltage;            // V
    float   ax, ay, az;         // m/s²
};

// ── Radio quality measured by the ground station ─────────────────────────────
struct RadioQuality {
    int rssi;       // dBm (negative, e.g. -80)
    int snr;        // dB  (e.g. 10)
};

// Decode `n_bytes` of big-endian binary into a zero-padded decimal string
// of length `n_digits`.  `out` must hold at least (n_digits + 1) chars.
void bytes_to_decimal(const uint8_t* in, int n_bytes,
                      char* out, int n_digits);

// Parse a 54-digit decimal string (LoRa packet format) into TelemetryData.
void parse_telemetry(const char* dec54, TelemetryData& t);

// Format a CSV line into `buf` (must be large enough, ~256 bytes is safe).
// Format:  $HAB,HHMMSS,lat,lon,spd,hdg,alt,volt,rssi,snr,ax,ay,az\n
// Returns the number of characters written (excluding null terminator).
int format_csv_line(const TelemetryData& t, const RadioQuality& rq,
                    char* buf, size_t buf_size);

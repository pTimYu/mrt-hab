#include "telemetry_decoder.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// ── Bytes → decimal string ───────────────────────────────────────────────────
// For each byte (MSB first): accumulator = accumulator × 256 + byte.
// Accumulator is stored as a fixed-width array of decimal digits.

void bytes_to_decimal(const uint8_t* in, int n_bytes,
                      char* out, int n_digits) {
    uint8_t digs[60];
    memset(digs, 0, sizeof(digs));

    for (int b = 0; b < n_bytes; b++) {
        uint16_t carry = in[b];
        for (int d = n_digits - 1; d >= 0; d--) {
            uint16_t val = (uint16_t)digs[d] * 256 + carry;
            digs[d] = val % 10;
            carry   = val / 10;
        }
    }

    for (int i = 0; i < n_digits; i++) {
        out[i] = '0' + digs[i];
    }
    out[n_digits] = '\0';
}

// ── Parse 54-digit decimal → TelemetryData ───────────────────────────────────
//
// LoRa digit layout (54 total):
//   [ 0.. 5]  Time       (6)  HHMMSS
//   [ 6..14]  Latitude   (9)  (lat × 1e5) + 1e8
//   [15..23]  Longitude  (9)  (lon × 1e5) + 1e8
//   [24..26]  Speed      (3)  speed × 10
//   [27..29]  Heading    (3)  heading
//   [30..35]  Height     (6)  altitude × 10
//   [36..38]  Voltage    (3)  voltage × 100
//   [39..43]  Accel X    (5)  (ax × 100) + 10000
//   [44..48]  Accel Y    (5)  (ay × 100) + 10000
//   [49..53]  Accel Z    (5)  (az × 100) + 10000

// Helper: extract N chars at offset, return as int32_t
static int32_t extract_int(const char* str, int pos, int len) {
    char tmp[10];
    memcpy(tmp, &str[pos], len);
    tmp[len] = '\0';
    return atol(tmp);
}

void parse_telemetry(const char* dec54, TelemetryData& t) {
    t.time_hhmmss = extract_int(dec54, 0, 6);
    t.latitude    = (extract_int(dec54, 6, 9) - 100000000L) / 100000.0f;
    t.longitude   = (extract_int(dec54, 15, 9) - 100000000L) / 100000.0f;
    t.speed_gps   = extract_int(dec54, 24, 3) / 10.0f;
    t.heading_gps = (int)extract_int(dec54, 27, 3);
    t.altitude    = extract_int(dec54, 30, 6) / 10.0f;
    t.voltage     = extract_int(dec54, 36, 3) / 100.0f;
    t.ax          = (extract_int(dec54, 39, 5) - 10000) / 100.0f;
    t.ay          = (extract_int(dec54, 44, 5) - 10000) / 100.0f;
    t.az          = (extract_int(dec54, 49, 5) - 10000) / 100.0f;
}

// ── Format CSV line ──────────────────────────────────────────────────────────
// $HAB,HHMMSS,lat,lon,spd,hdg,alt,volt,rssi,snr,ax,ay,az

int format_csv_line(const TelemetryData& t, const RadioQuality& rq,
                    char* buf, size_t buf_size) {
    // dtostrf is available on AVR / Arduino.
    // We build the string with snprintf for the integer parts,
    // then use dtostrf for the floats to avoid pulling in full printf float support.

    int pos = 0;

    // Header + time
    pos += snprintf(&buf[pos], buf_size - pos, "$HAB,%06ld,", (long)t.time_hhmmss);

    // Latitude (5 decimal places)
    dtostrf(t.latitude, 1, 5, &buf[pos]);
    pos = strlen(buf);
    buf[pos++] = ',';

    // Longitude (5 decimal places)
    dtostrf(t.longitude, 1, 5, &buf[pos]);
    pos = strlen(buf);
    buf[pos++] = ',';

    // Speed (1 decimal place)
    dtostrf(t.speed_gps, 1, 1, &buf[pos]);
    pos = strlen(buf);
    buf[pos++] = ',';

    // Heading (integer)
    pos += snprintf(&buf[pos], buf_size - pos, "%d,", t.heading_gps);

    // Altitude (1 decimal place)
    dtostrf(t.altitude, 1, 1, &buf[pos]);
    pos = strlen(buf);
    buf[pos++] = ',';

    // Voltage (2 decimal places)
    dtostrf(t.voltage, 1, 2, &buf[pos]);
    pos = strlen(buf);
    buf[pos++] = ',';

    // RSSI and SNR (integers, from ground station)
    pos += snprintf(&buf[pos], buf_size - pos, "%d,%d,", rq.rssi, rq.snr);

    // Accel X (2 decimal places)
    dtostrf(t.ax, 1, 2, &buf[pos]);
    pos = strlen(buf);
    buf[pos++] = ',';

    // Accel Y (2 decimal places)
    dtostrf(t.ay, 1, 2, &buf[pos]);
    pos = strlen(buf);
    buf[pos++] = ',';

    // Accel Z (2 decimal places)
    dtostrf(t.az, 1, 2, &buf[pos]);
    pos = strlen(buf);

    buf[pos] = '\0';
    return pos;
}

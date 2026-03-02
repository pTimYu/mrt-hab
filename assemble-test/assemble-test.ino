// ── HAB Flight Computer ───────────────────────────────────────────────────────
// Config  (must be first — defines TEST_MODE, ENABLE_BUZZER, pin numbers etc.)
#include "src/config.h"

// Arduino
#include <Wire.h>
#include <SPI.h>

// Sensors
#include "src/sensors/bmp390_sensor.h"
#include "src/sensors/lsm6dsm_sensor.h"
#include "src/sensors/adafruit_gps.h"

// I/O
#include "src/io/buzzer.h"

// Radio
#include "src/radio/lora_radio.h"

// Utilities
#include "src/utils/data_extractor.h"
#include "src/utils/data_saver.h"
#include "src/utils/kalman_filter.h"

// Flight logic
#include "src/flight/flight_controller.h"

// Mode Specified message
#if TEST_MODE
  #pragma message("TEST MODE ENABLED")
#else
  #pragma message("TEST MODE DISABLED")
#endif

// ── Global state ──────────────────────────────────────────────────────────────
static DataSet   data_set;
static uint32_t  last_loop_ms   = 0;
static uint32_t  last_lora_ms   = 0;
static bool      lora_ok        = false;

// ── Telemetry format ──────────────────────────────────────────────────────────
//
// 54-digit decimal string packed into 23 bytes (big-endian).
//
// Digit layout:
//   [ 0.. 5]  Time       (6 digits)  HHMMSS from GNSS
//   [ 6..14]  Latitude   (9 digits)  (lat × 1e5) + 1e8
//   [15..23]  Longitude  (9 digits)  (lon × 1e5) + 1e8
//   [24..26]  Speed      (3 digits)  speed_gps × 10
//   [27..29]  Heading    (3 digits)  heading_gps
//   [30..35]  Height     (6 digits)  Kalman altitude × 10
//   [36..38]  Voltage    (3 digits)  voltage × 100
//   [39..43]  Accel X    (5 digits)  (ax × 100) + 10000
//   [44..48]  Accel Y    (5 digits)  (ay × 100) + 10000
//   [49..53]  Accel Z    (5 digits)  (az × 100) + 10000
//
// 54 decimal digits → ceil(54 × log2(10) / 8) = ceil(179.38 / 8) = 23 bytes.
//
static const int  TELEM_DIGITS      = 54;
static const int  TELEM_PACKET_SIZE = 23;

// ── Clamping helper ───────────────────────────────────────────────────────────
static int32_t clamp_i(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ── Decimal → bytes (big-endian, repeated /256) ──────────────────────────────
// Converts an n_digits decimal string into n_bytes big-endian bytes.
static void decimal_to_bytes(const char* decimal, int n_digits,
                             uint8_t* out, int n_bytes) {
    uint8_t digs[TELEM_DIGITS];
    for (int i = 0; i < n_digits; i++) {
        digs[i] = decimal[i] - '0';
    }

    uint8_t tmp[TELEM_PACKET_SIZE];
    for (int b = 0; b < n_bytes; b++) {
        uint16_t remainder = 0;
        for (int i = 0; i < n_digits; i++) {
            uint16_t val = remainder * 10 + digs[i];
            digs[i] = (uint8_t)(val / 256);
            remainder = val % 256;
        }
        tmp[b] = (uint8_t)remainder;
    }

    // Reverse to big-endian
    for (int i = 0; i < n_bytes; i++) {
        out[i] = tmp[n_bytes - 1 - i];
    }
}

// ── Encode DataSet → 54-digit decimal → 23-byte binary ──────────────────────
static size_t pack_telemetry(const DataSet& d, uint8_t* buf) {
    char dec[TELEM_DIGITS + 1];
    int pos = 0;

    // [0-5]  Time HHMMSS (6 digits)
    int32_t t = clamp_i((int32_t)d.time_now, 0, 235959);
    sprintf(&dec[pos], "%06ld", (long)t);
    pos += 6;

    // [6-14]  Latitude (9 digits): (lat × 1e5) + 1e8
    int32_t lat_enc = clamp_i((int32_t)(d.latitude * 100000.0f + 100000000L),
                              0, 199999999L);
    sprintf(&dec[pos], "%09ld", (long)lat_enc);
    pos += 9;

    // [15-23]  Longitude (9 digits): (lon × 1e5) + 1e8
    int32_t lon_enc = clamp_i((int32_t)(d.longitude * 100000.0f + 100000000L),
                              0, 199999999L);
    sprintf(&dec[pos], "%09ld", (long)lon_enc);
    pos += 9;

    // [24-26]  GPS Speed (3 digits): speed × 10
    int32_t spd = clamp_i((int32_t)(d.speed_gps * 10.0f), 0, 999);
    sprintf(&dec[pos], "%03ld", (long)spd);
    pos += 3;

    // [27-29]  Heading (3 digits)
    int32_t hdg = clamp_i((int32_t)d.heading_gps, 0, 999);
    sprintf(&dec[pos], "%03ld", (long)hdg);
    pos += 3;

    // [30-35]  Height (6 digits): Kalman altitude × 10
    int32_t alt = clamp_i((int32_t)(d.altitude * 10.0f), 0, 999999);
    sprintf(&dec[pos], "%06ld", (long)alt);
    pos += 6;

    // [36-38]  Voltage (3 digits): voltage × 100
    int32_t volt = clamp_i((int32_t)(d.voltage * 100.0f), 0, 999);
    sprintf(&dec[pos], "%03ld", (long)volt);
    pos += 3;

    // [39-43]  Accel X (5 digits): (ax × 100) + 10000
    int32_t ax_e = clamp_i((int32_t)(d.ax * 100.0f) + 10000, 0, 19999);
    sprintf(&dec[pos], "%05ld", (long)ax_e);
    pos += 5;

    // [44-48]  Accel Y (5 digits): (ay × 100) + 10000
    int32_t ay_e = clamp_i((int32_t)(d.ay * 100.0f) + 10000, 0, 19999);
    sprintf(&dec[pos], "%05ld", (long)ay_e);
    pos += 5;

    // [49-53]  Accel Z (5 digits): (az × 100) + 10000
    int32_t az_e = clamp_i((int32_t)(d.az * 100.0f) + 10000, 0, 19999);
    sprintf(&dec[pos], "%05ld", (long)az_e);
    pos += 5;

    dec[TELEM_DIGITS] = '\0';

#if TEST_MODE
    Serial.print(F("[TELEM] dec="));
    Serial.println(dec);
#endif

    // Convert 54-digit decimal string → 23 bytes
    decimal_to_bytes(dec, TELEM_DIGITS, buf, TELEM_PACKET_SIZE);
    return TELEM_PACKET_SIZE;
}

// ── setup() ───────────────────────────────────────────────────────────────────
void setup() {

#if TEST_MODE
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000); // wait up to 3 s for USB serial
#endif

    // ── Hardware init ─────────────────────────────────────────────────────
    Wire.begin();
    Wire.setClock(I2C_CLOCK);
    delay(1000);

    buzzer_init();   // configures buzzer pin + LED pins

    // ── Sensor init — each failure beeps its device code and continues ────
    bool boot_ok = true;

    if (!bmp_init()) {
        buzzer_error(3);    // code 3 = BMP (pressure sensor)
        boot_ok = false;
    }

    if (!imu_init()) {
        buzzer_error(1);    // code 1 = IMU
        boot_ok = false;
    }

    if (!gps_init()) {
        buzzer_error(5);    // code 5 = GNSS
        boot_ok = false;
#if TEST_MODE
        Serial.println(F(">> GNSS init failed"));
#endif
    }

    if (!data_saver_init()) {
        buzzer_error(2);    // code 2 = SD card
        boot_ok = false;
    }

    // ── LoRa init ─────────────────────────────────────────────────────────
    lora_ok = lora_init();
    if (!lora_ok) {
        buzzer_error(4);    // code 4 = LoRa radio
        boot_ok = false;
    }

    // ── Filter + flight controller init ──────────────────────────────────
    kalman_init();
    fc_init();          // also fires actuator self-test in TEST_MODE

    // ── Boot result indication ────────────────────────────────────────────
    if (boot_ok) {
        buzzer_ok();
    } else {
        digitalWrite(LEDR, LOW);
    }

    last_loop_ms = millis();
    last_lora_ms = millis();
}

// ── loop() ────────────────────────────────────────────────────────────────────
void loop() {

    // ── 1. Compute dt ─────────────────────────────────────────────────────
    uint32_t now = millis();
    float dt = (now - last_loop_ms) / 1000.0f;
    last_loop_ms = now;

    // Guard against first-loop spike or millis() overflow
    if (dt <= 0.0f || dt > 1.0f) dt = LOOP_DELAY_MS / 1000.0f;

    // ── 2. Read raw sensors ───────────────────────────────────────────────
    data_extractor(data_set);

    // ── 3. Kalman filter ──────────────────────────────────────────────────
    kalman_update(data_set.bmp_altitude, data_set.bmp,
                  data_set.az,           data_set.imu,
                  dt);

    data_set.altitude          = kalman_get_altitude();
    data_set.vertical_velocity = kalman_get_velocity();

    // ── 4. Flight state machine ───────────────────────────────────────────
    fc_update(data_set);

    // ── 5. LoRa telemetry transmit ────────────────────────────────────────
    if (lora_ok && (now - last_lora_ms >= LORA_TX_INTERVAL_MS)) {
        //FlightStatus status = fc_get_status();
        //if (status != STNDBY) {
        // Tim: I don't think we need to wait until fly
        uint8_t packet[TELEM_PACKET_SIZE];
        size_t  pkt_len = pack_telemetry(data_set, packet);
        lora_send(packet, pkt_len);
        last_lora_ms = now;
        //}
    }

    delay(LOOP_DELAY_MS);
}

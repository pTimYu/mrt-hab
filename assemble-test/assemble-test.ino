// ── HAB Flight Computer ───────────────────────────────────────────────────────
// Config  (must be first — defines TEST_MODE, ENABLE_BUZZER, pin numbers etc.)
#include "src/config.h"

// Arduino
#include <Wire.h>
#include <SPI.h>

// Sensors
#include "src/sensors/bmp390_sensor.h"
#include "src/sensors/lsm6dsm_sensor.h"

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

// ── Telemetry packing ─────────────────────────────────────────────────────────
// Pack key DataSet fields into a compact binary buffer for LoRa transmission.
// All multi-byte values are little-endian (native on ARM Cortex-M / ESP32).
//
// Byte layout (32 bytes):
//   [ 0.. 3]  time               (int32)
//   [ 4.. 7]  altitude           (float)
//   [ 8..11]  vertical_velocity  (float)
//   [12..15]  temperature        (float)
//   [16..19]  pressure           (float)
//   [20..23]  ax                 (float)
//   [24..27]  ay                 (float)
//   [28..31]  az                 (float)
//   [32..35]  voltage            (float)
//   [36]      status flags       (uint8_t: bit0=bmp, bit1=imu, bit2=gnss)
//
// Total: 37 bytes

static const size_t TELEM_PACKET_SIZE = 37;

static size_t pack_telemetry(const DataSet& d, uint8_t* buf) {
    size_t pos = 0;

    memcpy(&buf[pos], &d.time,              sizeof(int32_t));  pos += 4;
    memcpy(&buf[pos], &d.altitude,          sizeof(float));    pos += 4;
    memcpy(&buf[pos], &d.vertical_velocity, sizeof(float));    pos += 4;
    memcpy(&buf[pos], &d.temperature,       sizeof(float));    pos += 4;
    memcpy(&buf[pos], &d.pressure,          sizeof(float));    pos += 4;
    memcpy(&buf[pos], &d.ax,               sizeof(float));    pos += 4;
    memcpy(&buf[pos], &d.ay,               sizeof(float));    pos += 4;
    memcpy(&buf[pos], &d.az,               sizeof(float));    pos += 4;
    memcpy(&buf[pos], &d.voltage,          sizeof(float));    pos += 4;

    uint8_t flags = 0;
    if (d.bmp)  flags |= 0x01;
    if (d.imu)  flags |= 0x02;
    if (d.gnss) flags |= 0x04;
    buf[pos++] = flags;

    return pos;  // should be TELEM_PACKET_SIZE
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

    if (!data_saver_init()) {
        buzzer_error(2);    // code 2 = SD card
        boot_ok = false;
    }

    // ── LoRa init ─────────────────────────────────────────────────────────
    lora_ok = lora_init();
    if (!lora_ok) {
        buzzer_error(5);    // code 5 = radio
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
    // Transmit at the configured interval during ASCENT, DESCENT, and LANDED.
    // Skip during STANDBY to avoid unnecessary radio traffic before flight.
    if (lora_ok && (now - last_lora_ms >= LORA_TX_INTERVAL_MS)) {
        FlightStatus status = fc_get_status();
        if (status != STNDBY) {
            uint8_t packet[TELEM_PACKET_SIZE];
            size_t  pkt_len = pack_telemetry(data_set, packet);
            lora_send(packet, pkt_len);
            last_lora_ms = now;
        }
    }

    delay(LOOP_DELAY_MS);
}

#pragma once

// ── Feature switches ──────────────────────────────────────────────────────────
// Set to 1 to enable, 0 to disable.
// TEST_MODE      : shorter flight limits + verbose serial output
// ENABLE_BUZZER  : drive the buzzer during dot/dash calls
#define TEST_MODE       1
#define ENABLE_BUZZER   0

// ── Flight termination limits ─────────────────────────────────────────────────
#if TEST_MODE
  #define MAX_FLIGHT_TIME    60000UL   // ms  — 1 minute
  #define MAX_FLIGHT_HEIGHT  10.0f     // m above ground
#else
  #define MAX_FLIGHT_TIME    2100000UL // ms  — 35 minutes
  #define MAX_FLIGHT_HEIGHT  5600.0f   // m above ground
#endif

// ── Flight detection thresholds ───────────────────────────────────────────────
#define TAKEOFF_THRESHOLD   1.0f   // m/s  vertical velocity → triggers ASCENT
#define LANDING_THRESHOLD   20     // consecutive stationary counts → LANDED
#define STANDBY_SETTLE_MS   20000  // ms after boot before takeoff detection starts

// ── I2C ───────────────────────────────────────────────────────────────────────
#define I2C_CLOCK           100000

// ── BMP390 ────────────────────────────────────────────────────────────────────
#define BMP_I2C_ADDR        0x76
#define BMP_SEA_LEVEL_HPA   1030.0f

// ── IMU (LSM6DSM) ─────────────────────────────────────────────────────────────
#define IMU_I2C_ADDR        0x6B
// ACCEL_FS_2G | ACCEL_FS_4G | ACCEL_FS_8G | ACCEL_FS_16G
#define IMU_ACCEL_RANGE     ACCEL_FS_2G
// GYRO_FS_250 | GYRO_FS_500 | GYRO_FS_1000 | GYRO_FS_2000
#define IMU_GYRO_RANGE      GYRO_FS_2000

// ── LoRa Radio ────────────────────────────────────────────────────────────────
#define LORA_RX_PIN         0      // MCU RX ← LoRa TX
#define LORA_TX_PIN         1      // MCU TX → LoRa RX
#define LORA_BAUD           9600   // HardwareSerial baud to LoRa module

// ── GNSS ──────────────────────────────────────────────────────────────────────
#define GNSS_RX             8
#define GNSS_TX             9
#define GNSS_BAUD           9600

// RF configuration string passed to AT+TEST=RFCFG
// Format: frequency(MHz),SF,BW(kHz),TxPreamble,RxPreamble,TxPower(dBm),CRC,IQ,NET
#define LORA_RF_CONFIG      "866,SF12,125,12,15,14,ON,OFF,OFF"

// How often to transmit telemetry (ms).  Set to 0 to send every loop.
#define LORA_TX_INTERVAL_MS 5000

// ── Pins ──────────────────────────────────────────────────────────────────────
// NOTE: On Seeed nRF52840 Sense, LEDs are active-LOW (HIGH = off, LOW = on)
#define ACTUATOR_PIN_1      6
#define ACTUATOR_PIN_2      7
#define BUZZER_PIN          2
#define SD_CS_PIN           4

// LED pins — Arduino Nano ESP32 board package defines LEDR / LEDG / LEDB
// as constexpr variables in pins_arduino.h.  They are active-LOW
// (LOW = on, HIGH = off).  No fallback needed here.

// ── Voltage divider (ADC) ─────────────────────────────────────────────────────
#define VOLTAGE_PIN         A7
#define ADC_REFERENCE_V     3.3f
#define ADC_RESOLUTION      4095.0f
#define VOLTAGE_SCALE       (60.0f / 19.0f)

// ── Radio ─────────────────────────────────────────────────────────────────────
#define RECEIVED_BIN        200
#define RECEIVED_BYTES      25
#define RECEIVED_DECIMAL    60

// ── System ────────────────────────────────────────────────────────────────────
#define SERIAL_BAUD         115200
#define LOOP_DELAY_MS       200

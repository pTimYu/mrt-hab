#pragma once

// ── Buzzer + LED indicator ────────────────────────────────────────────────────
//
// All functions respect the ENABLE_BUZZER compile-time switch.
// LEDs on Seeed nRF52840 Sense are active-LOW — the helpers handle this.
//
// Device error codes (pass to buzzer_error()):
//   0 — MPU6050   (legacy, kept for compatibility)
//   1 — IMU       (LSM6DSM)
//   2 — SD card
//   3 — BMP       (pressure sensor)
//   4 — HMC5883L  (reserved)
//   5 — GNSS      (reserved)
// ─────────────────────────────────────────────────────────────────────────────

// Call once in setup() to configure pins.
void buzzer_init();

// Morse primitives
void buzzer_dot();
void buzzer_dash();

// Play a digit (0–9) in Morse code.
void buzzer_digit(int digit);

// Play an error code: beeps the device number in Morse, repeats once.
// Blocks for the duration.
void buzzer_error(int device_code);

// Confirmation sequence: blue LED on + "OK" in Morse (--- -.-)
void buzzer_ok();

// "LAND" in Morse — played once; call every 60 s from the LANDED state.
void buzzer_land();

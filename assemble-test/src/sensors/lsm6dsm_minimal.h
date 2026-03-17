#pragma once
#include <Arduino.h>

// === Accelerometer range options ===
typedef enum {
    ACCEL_FS_2G  = 0x00,   // ±2g
    ACCEL_FS_4G  = 0x08,   // ±4g
    ACCEL_FS_8G  = 0x0C,   // ±8g
    ACCEL_FS_16G = 0x04    // ±16g
} AccelRange;

// === Gyroscope range options ===
typedef enum {
    GYRO_FS_250  = 0x00,   // ±250  °/s
    GYRO_FS_500  = 0x04,   // ±500  °/s
    GYRO_FS_1000 = 0x08,   // ±1000 °/s
    GYRO_FS_2000 = 0x0C    // ±2000 °/s
} GyroRange;

// === API ===
bool lsm6_init_minimal(uint8_t i2c_addr  = 0x6B,
                       AccelRange a_range = ACCEL_FS_2G,
                       GyroRange  g_range = GYRO_FS_2000);

bool lsm6_read_raw(int16_t &ax, int16_t &ay, int16_t &az,
                   int16_t &gx, int16_t &gy, int16_t &gz,
                   int16_t &t,  uint8_t i2c_addr = 0x6B);

// Unit conversion — automatically correct after init
float accel_ms2_from_raw(int16_t a_raw);
float gyro_dps_from_raw(int16_t g_raw);
float temp_c_from_raw(int16_t t_raw);
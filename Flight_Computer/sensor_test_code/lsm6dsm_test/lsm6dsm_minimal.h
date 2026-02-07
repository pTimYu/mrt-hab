#pragma once
#include <Arduino.h>

// I2C address
static const uint8_t LSM6_ADDR = 0x6B;

// API
bool lsm6_init_minimal();
bool lsm6_read_raw(int16_t &ax, int16_t &ay, int16_t &az,
                   int16_t &gx, int16_t &gy, int16_t &gz,
                   int16_t &t);

// unit conversion
float accel_ms2_from_raw(int16_t a_raw);
float gyro_dps_from_raw(int16_t g_raw);
float temp_c_from_raw(int16_t t_raw);

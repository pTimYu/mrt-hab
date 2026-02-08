#pragma once
#include <Arduino.h>


// API - 添加地址参数
bool lsm6_init_minimal(uint8_t i2c_addr = 0x6B);  // 默认值保持0x6B
bool lsm6_read_raw(int16_t &ax, int16_t &ay, int16_t &az,
                   int16_t &gx, int16_t &gy, int16_t &gz,
                   int16_t &t, uint8_t i2c_addr = 0x6B);  // 默认值保持0x6B

// unit conversion
float accel_ms2_from_raw(int16_t a_raw);
float gyro_dps_from_raw(int16_t g_raw);
float temp_c_from_raw(int16_t t_raw);
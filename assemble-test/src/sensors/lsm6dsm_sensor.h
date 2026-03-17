#pragma once
#include "lsm6dsm_minimal.h"

struct IMUData {
    float ax, ay, az;   // m/s²
    float gx, gy, gz;   // °/s
    float temperature;  // °C
    float roll, pitch;  // Orientation
};

bool imu_init();
bool imu_read(IMUData& data);
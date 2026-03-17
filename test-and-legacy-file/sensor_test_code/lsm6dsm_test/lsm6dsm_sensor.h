#pragma once
#include "lsm6dsm_minimal.h"

struct IMUData {
    float ax, ay, az;   // m/s²
    float gx, gy, gz;   // °/s
    float temperature;  // °C
};

bool imu_init();
bool imu_read(IMUData& data);
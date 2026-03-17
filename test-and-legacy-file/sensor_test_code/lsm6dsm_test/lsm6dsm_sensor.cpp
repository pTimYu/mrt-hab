#include "lsm6dsm_sensor.h"

#define IMU_I2C_ADDR 0x6B

bool imu_init() {
    return lsm6_init_minimal(IMU_I2C_ADDR);
}

bool imu_read(IMUData& data) {
    int16_t ax, ay, az, gx, gy, gz, t;
    if (!lsm6_read_raw(ax, ay, az, gx, gy, gz, t, IMU_I2C_ADDR)) return false;

    data.ax = accel_ms2_from_raw(ax);
    data.ay = accel_ms2_from_raw(ay);
    data.az = accel_ms2_from_raw(az);
    data.gx = gyro_dps_from_raw(gx);
    data.gy = gyro_dps_from_raw(gy);
    data.gz = gyro_dps_from_raw(gz);
    data.temperature = temp_c_from_raw(t);
    return true;
}
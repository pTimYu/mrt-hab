#include "lsm6dsm_sensor.h"
#include "../config.h"
#include "../utils/orientation_filter.h"

bool imu_init() {
    return lsm6_init_minimal(IMU_I2C_ADDR, IMU_ACCEL_RANGE, IMU_GYRO_RANGE);
}

bool imu_read(IMUData& data) {
    int16_t ax, ay, az, gx, gy, gz, t;
    if (!lsm6_read_raw(ax, ay, az, gx, gy, gz, t, IMU_I2C_ADDR)) return false;

    OrientationData data_temp = processIMU(accel_ms2_from_raw(ax), 
                      accel_ms2_from_raw(ay), 
                      accel_ms2_from_raw(az),
                      gyro_dps_from_raw(gx), 
                      gyro_dps_from_raw(gy), 0.01);
    
    data.ax = data_temp.ax;
    data.ay = data_temp.ay;
    data.az = data_temp.az;
    data.gx = data_temp.gx;
    data.gy = data_temp.gy;
    data.gz = gyro_dps_from_raw(gz);
    data.pitch = data_temp.pitch;
    data.roll = data_temp.roll;
    data.temperature = temp_c_from_raw(t);

    return true;
}
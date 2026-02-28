// Parameters Configuration Setup

#pragma once

// I2C(General)
#define I2C_CLOCK    100000


// BMP
#define BMP_I2C_ADDR        0x76
#define BMP_SEA_LEVEL_HPA   1013.25


// IMU
#define IMU_I2C_ADDR        0x6B
// Accelerometer range options:
// ACCEL_FS_2G, ACCEL_FS_4G, ACCEL_FS_8G, ACCEL_FS_16G
#define IMU_ACCEL_RANGE     ACCEL_FS_2G

// Gyroscope range options:
// GYRO_FS_250, GYRO_FS_500, GYRO_FS_1000, GYRO_FS_2000
#define IMU_GYRO_RANGE      GYRO_FS_2000


// SYSTEM
#define SERIAL_BAUD         115200
#define LOOP_DELAY_MS       200       // Should this be in "LoRa" Section?
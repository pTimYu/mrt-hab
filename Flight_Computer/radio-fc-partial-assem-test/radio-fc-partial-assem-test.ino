#include <Wire.h>
#include "bmp_sensor.h"
#include "imu_sensor.h"
#include "config.h"

// Sensor Structures
BMPData bmpData;
IMUData imuData;

// Variables
float bmp_temperature, bmp_pressure, bmp_altitude; // BMP
float imu_ax, imu_ay, imu_az; // IMU Acceleration
float imu_gx, imu_gy, imu_gz; // IMU Gyro


void setup() {
    Wire.begin();
    Wire.setClock(I2C_CLOCK);
    delay(1000);

    if (!bmp_init()) {
        // Placeholder, should activate the buzzer
        while (1);
    }

    if (!imu_init()) {
        // Placeholder, should activate the buzzer
        while (1);
    }

    // Buzzer response for successful initializing
}

void loop() {
    // BMP
    if (bmp_read(bmpData)) {
        bmp_altitude = bmpData.altitude;
        bmp_pressure = bmpData.pressure;
        bmp_temperature = bmpData.temperature;
    } else {
      // Is this reasonable?
      // If so, at least for the altitude, remember to add the condition to the Kalman Filter
        bmp_altitude = 0;
        bmp_pressure = 0;
        bmp_temperature = 100;
    }

    // IMU
    if (imu_read(imuData)) {
        imu_ax = imuData.ax;
        imu_ay = imuData.ay;
        imu_az = imuData.az;
        imu_gx = imuData.gx;
        imu_gy = imuData.gy;
        imu_gz = imuData.gz;
    } else {
      // Again, is this reasonable?
        imu_ax = 100;
        imu_ay = 100;
        imu_az = 100;
        imu_gx = 100;
        imu_gy = 100;
        imu_gz = 100;
    }

    delay(LOOP_DELAY_MS);
}
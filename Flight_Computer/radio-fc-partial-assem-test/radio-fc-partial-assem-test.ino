// Arduino Dependents

// Sensor Dependents
#include <Wire.h>
#include "bmp390_sensor.h"
#include "lsm6dsm_sensor.h"

// Data and Parameter
#include "data_extractor.h"
#include "config.h"

// Data Structures
DataSet data_set;

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

    delay(LOOP_DELAY_MS);
}
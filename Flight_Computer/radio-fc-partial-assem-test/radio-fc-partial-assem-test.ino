#include "module.h"

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
    data_extractor(data_set);
    delay(LOOP_DELAY_MS);
}
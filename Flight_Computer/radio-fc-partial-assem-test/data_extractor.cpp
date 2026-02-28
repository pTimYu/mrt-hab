// Dependent Library
#include <Wire.h>
#include "bmp390_sensor.h"
#include "lsm6dsm_sensor.h"

// Parameters
#include "config.h"

// Head
#include "data_extractor.h"


void data_extractor(DataSet& data){
    // Sensor Structures
    BMPData bmpData;
    IMUData imuData;

    // BMP
    if (bmp_read(bmpData)) {
        data.altitude = bmpData.altitude;
        data.pressure = bmpData.pressure;
        data.temperature = bmpData.temperature;
        data.bmp = true;
    } else {
        // Is this reasonable?
        // If so, at least for the altitude, remember to add the condition to the Kalman Filter
        data.altitude = 0;
        data.pressure = 0;
        data.temperature = 100;
        data.bmp = false;
    }

    // IMU
    if (imu_read(imuData)) {
        data.ax = imuData.ax;
        data.ay = imuData.ay;
        data.az = imuData.az;
        data.gx = imuData.gx;
        data.gy = imuData.gy;
        data.gz = imuData.gz;
        data.imu = true;
    } else {
        // Again, is this reasonable?
        data.ax = 100;
        data.ay = 100;
        data.az = 100;
        data.gx = 100;
        data.gy = 100;
        data.gz = 100;
        data.imu = false;
    }
}

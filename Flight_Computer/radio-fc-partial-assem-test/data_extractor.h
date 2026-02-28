#pragma once

struct DataSet {
    // BMP
    bool bmp;
    float temperature, pressure, altitude;

    // IMU
    bool imu;
    float ax, ay, az;
    float gx, gy, gz;
};

void data_extractor(DataSet& data);

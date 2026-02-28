#pragma once

extern serialib serial;

struct DataSet{
    int time;
    float latitude, longitude, speed_gps; int heading_gps; // GPS
    int height;
    float voltage;
    float RSSI, Gain;
    float accel_x, accel_y, accel_z;
};


bool data_receive_and_decompose(DataSet& data);
#pragma once

#include "serialib.h"

extern serialib serial;

struct DataSet{
    int time;                                              // HHMMSS from GNSS
    float latitude, longitude, speed_gps; int heading_gps; // GPS
    float altitude;                                        // metres (1 decimal)
    float voltage;
    float RSSI, Gain;                                      // measured by ground station
    float accel_x, accel_y, accel_z;
};


bool data_receive_and_decompose(DataSet& data);

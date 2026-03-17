#pragma once

#include "serialib.h"

extern serialib serial;

struct DataSet {
    int   time;                     // HHMMSS from GNSS
    float latitude, longitude;      // degrees
    float speed_gps;                // m/s
    int   heading_gps;              // degrees (0-359)
    float altitude;                 // metres (Kalman-filtered)
    float voltage;                  // V
    int   rssi;                     // dBm (negative)
    int   snr;                      // dB
    float accel_x, accel_y, accel_z;// m/s²
};

// Read one CSV line from serial, parse into DataSet.
// Expects lines in format: $HAB,HHMMSS,lat,lon,spd,hdg,alt,volt,rssi,snr,ax,ay,az
// Returns true on success, false on timeout or parse failure.
bool data_receive(DataSet& data);

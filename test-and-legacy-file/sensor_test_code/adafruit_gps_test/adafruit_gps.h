#pragma once

struct GnssData
{
    int time_now;                    // HHMMSS from GNSS (6 digits)
    int satellites;
    bool location_valid; float latitude; float longitude; // Location
    bool gps_altitude_valid; float gps_altitude;          // m (MSL)
    bool speed_gps_valid; float speed_gps;                // m/s
    bool heading_gps_valid; int   heading_gps;            // degrees (0–359)
};

bool gps_init();
void gps_read_data(GnssData& data);

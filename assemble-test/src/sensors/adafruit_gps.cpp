#include "adafruit_gps.h"
#include <TinyGPSPlus.h>
#include "../config.h"

TinyGPSPlus gps;

bool gps_init(){
    Serial1.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
    if (Serial1.available() > 0) return true;
    else return false;
}

void gps_read_data(GnssData& data){
    data.time_now = (gps.time.value() / 100) % 10000;
    
    data.location_valid = gps.location.isValid();
    data.latitude = gps.location.lat();
    data.longitude = gps.location.lng();

    data.gps_altitude_valid = gps.altitude.isValid();
    data.gps_altitude = gps.altitude.meters();

    data.speed_gps_valid = gps.speed.isValid();
    data.speed_gps = gps.speed.kmph() * (1 / 3.6f);

    data.heading_gps_valid = gps.course.isValid();
    data.heading_gps = gps.course.deg();
}
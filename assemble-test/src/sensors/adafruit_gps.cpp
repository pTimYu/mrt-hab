#include "adafruit_gps.h"
#include <TinyGPSPlus.h>
#include "../config.h"

// ── GPS parser instance ──────────────────────────────────────────────────────
static TinyGPSPlus gps;

// Use Serial2 for GPS — Serial1 is already taken by LoRa radio.
// On Arduino Nano ESP32 (ESP32-S3) there are 3 hardware UARTs:
//   Serial  = USB/UART0
//   Serial1 = LoRa (pins LORA_RX_PIN / LORA_TX_PIN)
//   Serial2 = GNSS (pins GNSS_RX / GNSS_TX)
#define GPS_SERIAL  Serial2

bool gps_init() {
    GPS_SERIAL.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);

    // Give the GPS module a moment to start sending data
    unsigned long start = millis();
    while (millis() - start < 2000) {
        if (GPS_SERIAL.available() > 0) return true;
        delay(10);
    }
    // No data after 2 s — module may not be connected
    return false;
}

void gps_read_data(GnssData& data) {
    // Feed all available bytes to the parser
    while (GPS_SERIAL.available() > 0) {
        gps.encode(GPS_SERIAL.read());
    }

    // Time: extract MMSS (4-digit integer)
    data.time_now = (gps.time.value() / 100) % 10000;

    // Location
    data.location_valid = gps.location.isValid();
    data.latitude  = gps.location.lat();
    data.longitude = gps.location.lng();

    // Altitude
    data.gps_altitude_valid = gps.altitude.isValid();
    data.gps_altitude = gps.altitude.meters();

    // Speed (TinyGPS++ gives km/h → convert to m/s)
    data.speed_gps_valid = gps.speed.isValid();
    data.speed_gps = gps.speed.kmph() * (1.0f / 3.6f);

    // Heading / course
    data.heading_gps_valid = gps.course.isValid();
    data.heading_gps = (int)gps.course.deg();
}

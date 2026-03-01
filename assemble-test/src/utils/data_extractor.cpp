#include "data_extractor.h"
#include "../config.h"
#include "../sensors/bmp390_sensor.h"
#include "../sensors/lsm6dsm_sensor.h"
#include "../sensors/adafruit_gps.h"

// ── Sentinel values written on sensor failure ─────────────────────────────────
// Using NaN would be ideal but causes issues with some loggers/parsers.
// Using out-of-range constants makes failures obvious in post-flight data.
// These are deliberately impossible real-world values.
static const float SENTINEL_ALTITUDE     = 99999.0f;  // m  — BMP or GPS failed
static const float SENTINEL_TEMPERATURE  =   999.0f;  // °C — BMP failed
static const float SENTINEL_PRESSURE     =     0.0f;  // hPa — BMP failed
static const float SENTINEL_ACCEL        =   999.0f;  // m/s² — IMU failed
static const float SENTINEL_GYRO         =   999.0f;  // °/s  — IMU failed
static const float SENTINEL_LOCAT        =   999.0f;  // °    — GPS failed
static const float SENTINEL_SPD          =   999.0f;  // m/s  — GPS failed
static const int   SENTINEL_HED          =   999;     // °    — GPS failed

void data_extractor(DataSet& data) {

    // ── Voltage ───────────────────────────────────────────────────────────
    int   raw  = analogRead(VOLTAGE_PIN);
    float v_a7 = (raw / ADC_RESOLUTION) * ADC_REFERENCE_V;
    data.voltage = v_a7 * VOLTAGE_SCALE;

    // ── BMP390 ────────────────────────────────────────────────────────────
    BMPData bmpData;
    if (bmp_read(bmpData)) {
        data.bmp_altitude = bmpData.altitude;
        data.pressure     = bmpData.pressure;
        data.temperature  = bmpData.temperature;
        data.bmp          = true;
    } else {
        data.bmp_altitude = SENTINEL_ALTITUDE;
        data.pressure     = SENTINEL_PRESSURE;
        data.temperature  = SENTINEL_TEMPERATURE;
        data.bmp          = false;
    }

    // ── IMU (LSM6DSM) ─────────────────────────────────────────────────────
    IMUData imuData;
    if (imu_read(imuData)) {
        data.ax  = imuData.ax;
        data.ay  = imuData.ay;
        data.az  = imuData.az;
        data.gx  = imuData.gx;
        data.gy  = imuData.gy;
        data.gz  = imuData.gz;
        data.imu = true;
    } else {
        data.ax  = SENTINEL_ACCEL;
        data.ay  = SENTINEL_ACCEL;
        data.az  = SENTINEL_ACCEL;
        data.gx  = SENTINEL_GYRO;
        data.gy  = SENTINEL_GYRO;
        data.gz  = SENTINEL_GYRO;
        data.imu = false;
    }

    // ── GPS ───────────────────────────────────────────────────────────────
    GnssData gnssData;
    gps_read_data(gnssData);            // <── was missing: populate gnssData

    data.time_now = gnssData.time_now;

    // Assume GPS is valid unless any critical field is bad
    bool gps_ok = true;

    if (gnssData.location_valid) {
        data.latitude  = gnssData.latitude;
        data.longitude = gnssData.longitude;
    } else {
        data.latitude  = SENTINEL_LOCAT;
        data.longitude = SENTINEL_LOCAT;
        gps_ok = false;
    }

    if (gnssData.gps_altitude_valid) {
        data.gps_altitude = gnssData.gps_altitude;
    } else {
        data.gps_altitude = SENTINEL_ALTITUDE;
        gps_ok = false;
    }

    if (gnssData.speed_gps_valid) {
        data.speed_gps = gnssData.speed_gps;
    } else {
        data.speed_gps = SENTINEL_SPD;
    }

    if (gnssData.heading_gps_valid) {
        data.heading_gps = gnssData.heading_gps;
    } else {
        data.heading_gps = SENTINEL_HED;
    }

    data.gps = gps_ok;                 // <── was missing: set the flag

    // ── Radio ─────────────────────────────────────────────────────────────
    // Placeholder — populate when LoRa RSSI/Gain is available.
    // data.RSSI, data.Gain
}

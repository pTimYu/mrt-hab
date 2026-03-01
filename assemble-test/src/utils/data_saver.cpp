#include "data_saver.h"
#include "../config.h"

#include <SdFat.h>

// ── File name ────────────────────────────────────────────────────────────────
// 8.3 format required by FAT
static const char FILE_NAME[] = "HAB_DATA.txt";

// ── Flush interval ───────────────────────────────────────────────────────────
// Flush buffered writes to card every N milliseconds.
// Flushing is much cheaper than close/re-open, but still has a small cost.
static const uint32_t FLUSH_INTERVAL_MS = 5000;

// ── Internal state ───────────────────────────────────────────────────────────
static SdFat  sd;
static SdFile logFile;
static uint32_t lastFlushMs = 0;

// ── Public API ───────────────────────────────────────────────────────────────

bool data_saver_init() {
    // Initialise SD at 4 MHz (conservative, matches sd_test.ino)
    if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(4))) {
        return false;
    }

    // Open (or create) the log file, always appending
    if (!logFile.open(FILE_NAME, O_RDWR | O_CREAT | O_AT_END)) {
        return false;
    }

    // Write a header line so the file is human-readable after recovery
    logFile.println(F("TIME\tVOLT\tTEMP\tPRES\tBMP_ALT\tALT\tVVEL\t"
                      "AX\tAY\tAZ\tGX\tGY\tGZ\t"
                      "LAT\tLON\tGPS_ALT\tSPD\tHDG\t"
                      "RSSI\tGAIN\tBMP_OK\tIMU_OK\tGNSS_OK"));
    logFile.flush();
    lastFlushMs = millis();

    return true;
}

bool data_saver_write(const DataSet& data) {
    if (!logFile.isOpen()) {
        // Try to re-open if something went wrong
        if (!logFile.open(FILE_NAME, O_RDWR | O_CREAT | O_AT_END)) {
            return false;
        }
    }

    // ── Build tab-separated row ───────────────────────────────────────────
    // Using multiple print() calls avoids a large stack buffer and is safe
    // on memory-constrained MCUs.

    // General
    logFile.print(data.time);           logFile.print('\t');
    logFile.print(data.voltage, 3);     logFile.print('\t');

    // BMP
    logFile.print(data.temperature, 2); logFile.print('\t');
    logFile.print(data.pressure, 2);    logFile.print('\t');
    logFile.print(data.bmp_altitude, 2);logFile.print('\t');

    // Kalman filtered outputs
    logFile.print(data.altitude, 2);           logFile.print('\t');
    logFile.print(data.vertical_velocity, 3);  logFile.print('\t');

    // IMU
    logFile.print(data.ax, 3);          logFile.print('\t');
    logFile.print(data.ay, 3);          logFile.print('\t');
    logFile.print(data.az, 3);          logFile.print('\t');
    logFile.print(data.gx, 3);          logFile.print('\t');
    logFile.print(data.gy, 3);          logFile.print('\t');
    logFile.print(data.gz, 3);          logFile.print('\t');

    // GPS
    logFile.print(data.latitude,  6);   logFile.print('\t');
    logFile.print(data.longitude, 6);   logFile.print('\t');
    logFile.print(data.gps_altitude, 2);logFile.print('\t');
    logFile.print(data.speed_gps, 2);   logFile.print('\t');
    logFile.print(data.heading_gps);    logFile.print('\t');

    // Radio
    logFile.print(data.RSSI, 1);        logFile.print('\t');
    logFile.print(data.Gain, 1);        logFile.print('\t');

    // Sensor health flags
    logFile.print(data.bmp  ? 1 : 0);  logFile.print('\t');
    logFile.print(data.imu  ? 1 : 0);  logFile.print('\t');
    logFile.print(data.gnss ? 1 : 0);

    logFile.println(); // end of row

    // ── Periodic flush ────────────────────────────────────────────────────
    uint32_t now = millis();
    if (now - lastFlushMs >= FLUSH_INTERVAL_MS) {
        logFile.flush();
        lastFlushMs = now;
    }

    return true;
}

void data_saver_flush() {
    if (logFile.isOpen()) {
        logFile.flush();
        lastFlushMs = millis();
    }
}

void data_saver_close() {
    if (logFile.isOpen()) {
        logFile.flush();
        logFile.close();
    }
}

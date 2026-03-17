#include "data_receive.h"
#include <cstdio>
#include <cstring>

// Maximum length of a CSV line (generous buffer)
static const int MAX_LINE_LEN = 512;

// Read one newline-terminated line from the serial port.
// Returns the number of characters read, or 0 on timeout.
static int read_line(char* buf, int max_len, unsigned int timeout_ms) {
    int pos = 0;
    char c;

    while (pos < max_len - 1) {
        int ret = serial.readChar(&c, timeout_ms);
        if (ret != 1) {
            // Timeout or error
            buf[pos] = '\0';
            return 0;
        }
        if (c == '\n') {
            buf[pos] = '\0';
            return pos;
        }
        if (c == '\r') {
            continue;   // skip carriage return
        }
        buf[pos++] = c;
    }

    buf[pos] = '\0';
    return pos;
}

bool data_receive(DataSet& data) {
    char line[MAX_LINE_LEN];

    // Read one line (5 second timeout)
    int len = read_line(line, MAX_LINE_LEN, 5000);
    if (len == 0) return false;

    // Skip lines that don't start with $HAB (status messages, etc.)
    if (strncmp(line, "$HAB,", 5) != 0) return false;

    // Parse CSV fields after the "$HAB," prefix
    int matched = sscanf(line + 5,
        "%d,%f,%f,%f,%d,%f,%f,%d,%d,%f,%f,%f",
        &data.time,
        &data.latitude,
        &data.longitude,
        &data.speed_gps,
        &data.heading_gps,
        &data.altitude,
        &data.voltage,
        &data.rssi,
        &data.snr,
        &data.accel_x,
        &data.accel_y,
        &data.accel_z
    );

    return (matched == 12);
}

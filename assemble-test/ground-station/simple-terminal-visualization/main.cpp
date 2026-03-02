#include <iostream>
#include <string>
#include <vector>
#include <cstdio>

#include "data-receive/data_receive.h"
#include "data-receive/serialib.h"

// For Sleep/usleep
#if defined(_WIN32) || defined(_WIN64)
    #include <windows.h>
    #define SLEEP(ms) Sleep(ms)
#else
    #include <unistd.h>
    #define SLEEP(ms) usleep((ms) * 1000)
#endif

serialib serial;
DataSet received_data;

// Scan common serial port names and return the first one that opens
std::string findArduinoPort() {
    serialib tempSerial;
    std::vector<std::string> portNames;

#if defined(_WIN32) || defined(_WIN64)
    for (int i = 1; i <= 20; ++i)
        portNames.push_back("\\\\.\\COM" + std::to_string(i));
#else
    for (int i = 0; i < 10; ++i) {
        portNames.push_back("/dev/ttyACM" + std::to_string(i));
        portNames.push_back("/dev/ttyUSB" + std::to_string(i));
    }
#endif

    std::cout << "Scanning for Arduino..." << std::endl;

    for (const std::string& port : portNames) {
        if (tempSerial.openDevice(port.c_str(), 115200) == 1) {
            SLEEP(2000);    // wait for Arduino to reboot
            tempSerial.closeDevice();
            return port;
        }
    }
    return "";
}

int main() {
    // --- Step 1: Detect the port ---
    std::string detectedPort = findArduinoPort();

    if (detectedPort.empty()) {
        std::cerr << "Error: No Arduino detected on any port!" << std::endl;
        return 1;
    }

    // --- Step 2: Open the detected port ---
    if (serial.openDevice(detectedPort.c_str(), 115200) != 1) {
        std::cerr << "Error: Could not re-open " << detectedPort << std::endl;
        return 1;
    }

    std::cout << "Connected to Arduino on " << detectedPort << std::endl;

    // --- Step 3: Main data loop ---
    while (true) {
        if (data_receive(received_data)) {
            std::cout << "--- Current Data Set ---" << std::endl;

            // Time as HH:MM:SS
            int t  = received_data.time;
            int hh = t / 10000;
            int mm = (t / 100) % 100;
            int ss = t % 100;
            printf("Time:        %02d:%02d:%02d\n", hh, mm, ss);

            printf("Lat/Lon:     %.5f, %.5f\n",
                   received_data.latitude, received_data.longitude);
            printf("Speed/Hdg:   %.1f m/s / %d deg\n",
                   received_data.speed_gps, received_data.heading_gps);
            printf("Altitude:    %.1f m\n", received_data.altitude);
            printf("Voltage:     %.2f V\n", received_data.voltage);
            printf("RSSI:        %d dBm\n", received_data.rssi);
            printf("SNR:         %d dB\n",  received_data.snr);
            printf("Accel X/Y/Z: %.2f, %.2f, %.2f m/s2\n",
                   received_data.accel_x,
                   received_data.accel_y,
                   received_data.accel_z);

            std::cout << "------------------------" << std::endl;
        }
    }

    serial.closeDevice();
    return 0;
}

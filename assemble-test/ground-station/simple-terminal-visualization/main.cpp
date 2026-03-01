#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>

#include "data-receive-and-decomposition/data_receive_and_decompose.h"
#include "data-receive-and-decomposition/serialib.h"

// For Sleep/usleep
#if defined(_WIN32) || defined(_WIN64)
    #include <windows.h>
    #define SLEEP(ms) Sleep(ms)
#else
    #include <unistd.h>
    #define SLEEP(ms) usleep(ms * 1000)
#endif

serialib serial;
DataSet received_data;

// Function to find the Arduino port automatically
std::string findArduinoPort() {
    serialib tempSerial;
    std::vector<std::string> portNames;

    // 1. Generate list of potential ports
#if defined(_WIN32) || defined(_WIN64)
    for (int i = 1; i <= 20; ++i) portNames.push_back("\\\\.\\COM" + std::to_string(i));
#else
    for (int i = 0; i < 10; ++i) {
        portNames.push_back("/dev/ttyACM" + std::to_string(i));
        portNames.push_back("/dev/ttyUSB" + std::to_string(i));
    }
#endif

    std::cout << "Scanning for Arduino..." << std::endl;

    for (const std::string& port : portNames) {
        if (tempSerial.openDevice(port.c_str(), 115200) == 1) {
            // Wait for Arduino to reboot after opening the port
            SLEEP(2000); 

            // Optional: Send a '?' character to ask "Are you there?"
            tempSerial.writeChar('?');
            
            char response[20];
            // If Arduino responds with "ACK" (you must program Arduino to do this)
            // Or simply assume if it opened, it's the right one:
            tempSerial.closeDevice();
            return port; 
        }
    }
    return "";
}

int main() {
    // --- Step 1: Detect the Port ---
    std::string detectedPort = findArduinoPort();

    if (detectedPort.empty()) {
        std::cerr << "Error: No Arduino detected on any port!" << std::endl;
        return 1;
    }

    // --- Step 2: Open the Detected Port ---
    if (serial.openDevice(detectedPort.c_str(), 115200) != 1) {
        std::cerr << "Error: Could not re-open " << detectedPort << std::endl;
        return 1;
    }

    std::cout << "Connected to Arduino on " << detectedPort << std::endl;

    // --- Step 3: Main Data Loop ---
    while (true) {
        if (data_receive_and_decompose(received_data)) {
            std::cout << "--- Current Data Set ---" << std::endl;
            std::cout << "Time:      " << received_data.time << std::endl;
            std::cout << "Lat/Long:  " << received_data.latitude << ", " << received_data.longitude << std::endl;
            std::cout << "Speed/Hdg: " << received_data.speed_gps << " / " << received_data.heading_gps << std::endl;
            std::cout << "Altitude:    " << received_data.altitude << std::endl;
            std::cout << "Voltage:   " << received_data.voltage << "V" << std::endl;
            std::cout << "Accel X/Y/Z: " 
                      << received_data.accel_x << ", " 
                      << received_data.accel_y << ", " 
                      << received_data.accel_z << std::endl;
            std::cout << "------------------------" << std::endl;
        }
    }

    serial.closeDevice();
    return 0;
}
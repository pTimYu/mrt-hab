#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include "data_receive_and_decompose.h"
#include "serialib.h"
#include "../../../src/config.h"

// Input: Serial Port
// Output: a boolean value to see if the data correctly collected.
// Functionality: Receive the binary data from ground station board,
//                then convert it to decimal place, decompose and save 
//                it to the strut.
//
// PC 60-digit layout (from ground station):
//   [ 0.. 5]  Time       (6)  HHMMSS
//   [ 6..14]  Latitude   (9)  (lat × 1e5) + 1e8
//   [15..23]  Longitude  (9)  (lon × 1e5) + 1e8
//   [24..26]  Speed      (3)  speed × 10
//   [27..29]  Heading    (3)  heading
//   [30..35]  Height     (6)  altitude × 10
//   [36..38]  Voltage    (3)  voltage × 100
//   [39..41]  RSSI       (3)  |RSSI| in dBm  (from ground station)
//   [42..44]  SNR/Gain   (3)  (SNR × 10) + 200  (from ground station)
//   [45..49]  Accel X    (5)  (ax × 100) + 10000
//   [50..54]  Accel Y    (5)  (ay × 100) + 10000
//   [55..59]  Accel Z    (5)  (az × 100) + 10000

std::string bytesToDecimal(unsigned char* data, int len) {
    std::string decimal = "0";

    for (int i = 0; i < len; i++) {
        // Step 1: Start with the value of the current byte
        int carry = data[i]; 

        // Step 2: Multiply the existing decimal string by 256 and add the byte
        for (int j = decimal.size() - 1; j >= 0; j--) {
            int val = (decimal[j] - '0') * 256 + carry;
            decimal[j] = (val % 10) + '0';
            carry = val / 10;
        }

        // Step 3: Handle any remaining carry by adding new digits to the front
        while (carry) {
            decimal.insert(0, 1, (carry % 10) + '0');
            carry /= 10;
        }
    }

    return decimal;
}

void mapToDataSet(std::string fullDecimal, DataSet& data) {
    const int totalExpected = RECEIVED_DECIMAL;     // 60
    if ((int)fullDecimal.length() < totalExpected) {
        fullDecimal.insert(0, totalExpected - fullDecimal.length(), '0');
    }

    int pos = 0;
    auto getNext = [&](int len) -> std::string {
        std::string sub = fullDecimal.substr(pos, len);
        pos += len;
        return sub;
    };

    data.time        = std::stoi(getNext(6));                            // HHMMSS
    data.latitude    = (std::stof(getNext(9)) - 1e8f) / 1e5f;           // degrees
    data.longitude   = (std::stof(getNext(9)) - 1e8f) / 1e5f;           // degrees
    data.speed_gps   = std::stof(getNext(3)) / 10.0f;                   // m/s
    data.heading_gps = std::stoi(getNext(3));                            // degrees
    data.altitude    = std::stof(getNext(6)) / 10.0f;                   // metres (1 dp)
    data.voltage     = std::stof(getNext(3)) / 100.0f;                  // V

    // RSSI: stored as |RSSI| → negate to get conventional dBm
    data.RSSI        = -std::stof(getNext(3));                           // dBm

    // Gain / SNR: stored as (SNR × 10) + 200 → reverse offset
    data.Gain        = (std::stof(getNext(3)) - 200.0f) / 10.0f;        // dB

    // Acceleration: stored as (value × 100) + 10000
    data.accel_x     = (std::stof(getNext(5)) - 10000.0f) / 100.0f;     // m/s²
    data.accel_y     = (std::stof(getNext(5)) - 10000.0f) / 100.0f;     // m/s²
    data.accel_z     = (std::stof(getNext(5)) - 10000.0f) / 100.0f;     // m/s²
}

bool data_receive_and_decompose(DataSet& data){
    unsigned char received[RECEIVED_BYTES];
    // Read Bytes
    if(serial.readBytes(received, RECEIVED_BYTES, 2000, 1000) < RECEIVED_BYTES){
        return false;
    }
    std::string decimal = bytesToDecimal(received, RECEIVED_BYTES);
    // Decomposition
    mapToDataSet(decimal, data);
    return true;
}

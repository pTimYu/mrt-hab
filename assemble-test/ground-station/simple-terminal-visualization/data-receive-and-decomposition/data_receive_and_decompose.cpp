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

std::vector<int> digits = {4, 9, 9, 3, 3, 5, 3, 3, 3, 6, 6, 6};

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
    const int totalExpected = 60;
    if (fullDecimal.length() < totalExpected) {
        fullDecimal.insert(0, totalExpected - fullDecimal.length(), '0');
    }

    int pos = 0;
    auto getNext = [&](int len) {
        std::string sub = fullDecimal.substr(pos, len);
        pos += len;
        return sub;
    };

    data.time        = std::stoi(getNext(4));                            // MMSS
    data.latitude    = (std::stof(getNext(9)) - 1e8f) / 1e5f;           // restore degrees
    data.longitude   = (std::stof(getNext(9)) - 1e8f) / 1e5f;
    data.speed_gps   = std::stof(getNext(3)) / 10.0f;                   // restore m/s
    data.heading_gps = std::stoi(getNext(3));
    data.altitude    = std::stoi(getNext(5));                            // metres, integer
    data.voltage     = std::stof(getNext(3)) / 100.0f;                  // restore V

    // RSSI: stored as |RSSI| in dBm (e.g. 080 → 80 dBm signal strength)
    // Negate to get conventional dBm if desired, or keep as positive magnitude
    data.RSSI        = -std::stof(getNext(3));                           // dBm (negative)

    // Gain / SNR: stored as (SNR × 10 + 200)
    // Reverse the offset: SNR = (encoded - 200) / 10.0
    data.Gain        = (std::stof(getNext(3)) - 200.0f) / 10.0f;        // dB

    data.accel_x     = (std::stof(getNext(6)) - 100000.0f) / 1000.0f;   // restore m/s²
    data.accel_y     = (std::stof(getNext(6)) - 100000.0f) / 1000.0f;
    data.accel_z     = (std::stof(getNext(6)) - 100000.0f) / 1000.0f;
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

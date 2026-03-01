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
    // 1. Ensure the string is exactly 60 digits (left-pad with zeros if needed)
    const int totalExpected = 60;
    if (fullDecimal.length() < totalExpected) {
        fullDecimal.insert(0, totalExpected - fullDecimal.length(), '0');
    }

    // 2. Define the lengths provided in your 'digits' list
    // indices: 0  1  2  3  4  5  6  7  8  9  10 11
    // digits: [4, 9, 9, 3, 3, 5, 3, 3, 3, 6, 6, 6]

    int pos = 0;

    // Helper lambda to extract a substring and move the pointer
    auto getNext = [&](int len) {
        std::string sub = fullDecimal.substr(pos, len);
        pos += len;
        return sub;
    };

    // 3. Map string segments to Struct fields
    // Use std::stoi for int and std::stof for float
    
    data.time        = std::stoi(getNext(4));  // 4 digits
    data.latitude    = std::stof(getNext(9));  // 9 digits
    data.longitude   = std::stof(getNext(9));  // 9 digits
    data.speed_gps   = std::stof(getNext(3));  // 3 digits
    data.heading_gps = std::stoi(getNext(3));  // 3 digits
    data.altitude    = std::stoi(getNext(5));  // 5 digits
    data.voltage     = std::stof(getNext(3));  // 3 digits
    data.RSSI        = std::stof(getNext(3));  // 3 digits
    data.Gain        = std::stof(getNext(3));  // 3 digits
    data.accel_x     = std::stof(getNext(6));  // 6 digits
    data.accel_y     = std::stof(getNext(6));  // 6 digits
    data.accel_z     = std::stof(getNext(6));  // 6 digits

}

bool data_receive_and_decompose(DataSet& data){
    unsigned char received[received_bytes];
    // Read Bytes
    if(serial.readBytes(received, received_bytes, 2000, 1000) < received_bytes){
        return false;
    }
    std::string decimal = bytesToDecimal(received, received_bytes);
    // Decomposition
    mapToDataSet(decimal, data);
    return true;
}
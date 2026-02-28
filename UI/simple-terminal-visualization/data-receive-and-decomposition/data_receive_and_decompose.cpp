#include <stdio.h>
#include <unistd.h>
#include "data_receive_and_decompose.h"
#include "serialib.h"

// Parameters
int received_bytes = 25; // The byte number we will have for the
int received_decimal = 60; // The converted decimal place



// Input: Serial Port
// Output: a boolean value to see if the data correctly collected.
// Functionality: Receive the binary data from ground station board,
//                then convert it to decimal place, decompose and save 
//                it to the strut.
bool data_receive_and_decompose(DataSet& data, char* SERIAL_PORT){
    unsigned char received[received_bytes];
    serial.readBytes(received, received_bytes, 2000, 1000);
}
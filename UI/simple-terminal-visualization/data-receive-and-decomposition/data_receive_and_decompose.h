#pragma once

extern serialib serial;

struct DataSet{
};


bool data_receive_and_decompose(DataSet& data, char* SERIAL_PORT);
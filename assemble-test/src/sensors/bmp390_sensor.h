#pragma once

#include <Adafruit_BMP3XX.h>

struct BMPData {
    float temperature;   // °C
    float pressure;      // hPa
    float altitude;      // m
};

bool bmp_init();
bool bmp_read(BMPData& data);
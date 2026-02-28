#include "bmp_sensor.h"
#include "config.h"

static Adafruit_BMP3XX bmp;

bool bmp_init() {
    if (!bmp.begin_I2C(BMP_I2C_ADDR)) return false;
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    return true;
}

bool bmp_read(BMPData& data) {
    if (!bmp.performReading()) return false;

    data.temperature = bmp.temperature;
    data.pressure    = bmp.pressure / 100.0;
    data.altitude    = bmp.readAltitude(BMP_SEA_LEVEL_HPA);
    return true;
}
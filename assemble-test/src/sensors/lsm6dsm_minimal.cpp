#include "lsm6dsm_minimal.h"
#include <Wire.h>

/* ===== Register map ===== */
static const uint8_t REG_WHO_AM_I   = 0x0F;
static const uint8_t REG_CTRL1_XL   = 0x10;
static const uint8_t REG_CTRL2_G    = 0x11;
static const uint8_t REG_CTRL3_C    = 0x12;
static const uint8_t REG_OUT_TEMP_L = 0x20;

/* ===== CTRL3 bits ===== */
static const uint8_t CTRL3_SW_RESET = (1 << 0);
static const uint8_t CTRL3_BDU      = (1 << 6);
static const uint8_t CTRL3_IF_INC   = (1 << 2);

/* ===== Sensitivity lookup tables ===== */
// Units: mg/LSB → will convert to m/s²
static const float ACCEL_SENSITIVITY[] = {
    0.061f,    // ±2g
    0.122f,    // ±4g
    0.244f,    // ±8g
    0.488f     // ±16g  (index 3, but FS bits are non-linear, see below)
};

// Units: mdps/LSB
static const float GYRO_SENSITIVITY[] = {
    8.75f,     // ±250  °/s
    17.50f,    // ±500  °/s
    35.00f,    // ±1000 °/s
    70.00f     // ±2000 °/s
};

/* ===== Active sensitivity (set during init) ===== */
static float s_accel_sens = 0.061f;   // default ±2g
static float s_gyro_sens  = 70.00f;   // default ±2000 °/s

/* ===== I2C helpers ===== */
static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission(true) == 0;
}

static bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    size_t got = Wire.requestFrom(addr, (uint8_t)n);
    if (got != n) return false;
    for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
}

static int16_t le16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* ===== Sensitivity index helpers ===== */
// AccelRange enum values are the raw FS bits, map to sensitivity index
static int accelRangeToIndex(AccelRange r) {
    switch (r) {
        case ACCEL_FS_2G:  return 0;
        case ACCEL_FS_4G:  return 1;
        case ACCEL_FS_8G:  return 2;
        case ACCEL_FS_16G: return 3;
        default:           return 0;
    }
}

// GyroRange enum values are the raw FS bits, map to sensitivity index
static int gyroRangeToIndex(GyroRange r) {
    switch (r) {
        case GYRO_FS_250:  return 0;
        case GYRO_FS_500:  return 1;
        case GYRO_FS_1000: return 2;
        case GYRO_FS_2000: return 3;
        default:           return 3;
    }
}

/* ===== Public API ===== */
bool lsm6_init_minimal(uint8_t i2c_addr, AccelRange a_range, GyroRange g_range) {
    uint8_t who = 0;
    if (!i2cRead(i2c_addr, REG_WHO_AM_I, &who, 1)) return false;

    // Software reset
    i2cWrite8(i2c_addr, REG_CTRL3_C, CTRL3_SW_RESET);
    delay(50);

    // Enable BDU + auto address increment
    i2cWrite8(i2c_addr, REG_CTRL3_C, CTRL3_BDU | CTRL3_IF_INC);

    // CTRL1_XL: ODR=104Hz (0x40) | accel FS bits
    uint8_t ctrl1 = 0x40 | (uint8_t)a_range;
    i2cWrite8(i2c_addr, REG_CTRL1_XL, ctrl1);

    // CTRL2_G: ODR=104Hz (0x40) | gyro FS bits
    uint8_t ctrl2 = 0x40 | (uint8_t)g_range;
    i2cWrite8(i2c_addr, REG_CTRL2_G, ctrl2);

    // Store active sensitivity for unit conversion
    s_accel_sens = ACCEL_SENSITIVITY[accelRangeToIndex(a_range)];
    s_gyro_sens  = GYRO_SENSITIVITY[gyroRangeToIndex(g_range)];

    return true;
}

bool lsm6_read_raw(int16_t &ax, int16_t &ay, int16_t &az,
                   int16_t &gx, int16_t &gy, int16_t &gz,
                   int16_t &t,  uint8_t i2c_addr) {
    uint8_t buf[14];
    if (!i2cRead(i2c_addr, REG_OUT_TEMP_L, buf, 14)) return false;

    t  = le16(&buf[0]);
    gx = le16(&buf[2]);
    gy = le16(&buf[4]);
    gz = le16(&buf[6]);
    ax = le16(&buf[8]);
    ay = le16(&buf[10]);
    az = le16(&buf[12]);
    return true;
}

/* ===== Unit conversion ===== */
float accel_ms2_from_raw(int16_t a) {
    // mg/LSB → m/s²
    return a * s_accel_sens / 1000.0f * 9.80665f;
}

float gyro_dps_from_raw(int16_t g) {
    // mdps/LSB → °/s
    return g * s_gyro_sens / 1000.0f;
}

float temp_c_from_raw(int16_t t_raw) {
    return ((float)t_raw / 256.0f) + 25.0f;
}
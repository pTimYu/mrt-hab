#include "lsm6dsm_minimal.h"
#include <Wire.h>

/* ===== register map ===== */
static const uint8_t REG_WHO_AM_I   = 0x0F;
static const uint8_t REG_CTRL1_XL   = 0x10;
static const uint8_t REG_CTRL2_G    = 0x11;
static const uint8_t REG_CTRL3_C    = 0x12;
static const uint8_t REG_STATUS_REG = 0x1E;
static const uint8_t REG_OUT_TEMP_L = 0x20;

/* ===== CTRL3 bits ===== */
static const uint8_t CTRL3_SW_RESET = (1 << 0);
static const uint8_t CTRL3_BDU      = (1 << 6);
static const uint8_t CTRL3_IF_INC   = (1 << 2);

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

  size_t read = Wire.requestFrom(addr, (uint8_t)n);
  if (read != n) return false;

  for (size_t i = 0; i < n; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

static int16_t le16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* ===== public API ===== */
bool lsm6_init_minimal() {
  uint8_t who = 0;
  if (!i2cRead(LSM6_ADDR, REG_WHO_AM_I, &who, 1)) {
    return false;
  }

  i2cWrite8(LSM6_ADDR, REG_CTRL3_C, CTRL3_SW_RESET);
  delay(50);

  i2cWrite8(LSM6_ADDR, REG_CTRL3_C, CTRL3_BDU | CTRL3_IF_INC);
  i2cWrite8(LSM6_ADDR, REG_CTRL1_XL, 0x40);
  i2cWrite8(LSM6_ADDR, REG_CTRL2_G,  0x4C);

  return true;
}


bool lsm6_read_raw(int16_t &ax, int16_t &ay, int16_t &az,
                   int16_t &gx, int16_t &gy, int16_t &gz,
                   int16_t &t) {
  uint8_t buf[14];
  if (!i2cRead(LSM6_ADDR, REG_OUT_TEMP_L, buf, 14)) return false;

  t  = le16(&buf[0]);
  gx = le16(&buf[2]);
  gy = le16(&buf[4]);
  gz = le16(&buf[6]);
  ax = le16(&buf[8]);
  ay = le16(&buf[10]);
  az = le16(&buf[12]);
  return true;
}

/* ===== unit conversion ===== */
float accel_ms2_from_raw(int16_t a) {
  return a * 0.061f / 1000.0f * 9.80665f;
}

float gyro_dps_from_raw(int16_t g) {
  return g * 70.0f / 1000.0f;
}

float temp_c_from_raw(int16_t t) {
  return 25.0f + t / 16.0f;
}

#include <Wire.h>
#include "lsm6dsm_minimal.h"

static const uint8_t LSM6_ADDR = 0x6B;  // LSM I2C Address

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  Wire.setClock(100000);

  if (!lsm6_init_minimal(LSM6_ADDR)) {
    Serial.println("LSM6 init FAILED");
    while (1) delay(1000);
  }
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz, t;

  if (!lsm6_read_raw(ax, ay, az, gx, gy, gz, t, LSM6_ADDR)) {
    delay(50);
    return;
  }

  float ax_ms2 = accel_ms2_from_raw(ax);
  float ay_ms2 = accel_ms2_from_raw(ay);
  float az_ms2 = accel_ms2_from_raw(az);

  float gx_dps = gyro_dps_from_raw(gx);
  float gy_dps = gyro_dps_from_raw(gy);
  float gz_dps = gyro_dps_from_raw(gz);

  float temp_c = temp_c_from_raw(t);

  // ===== Serial Plotter format =====
  Serial.print("Ax:"); Serial.print(ax_ms2, 3);
  Serial.print("\tAy:"); Serial.print(ay_ms2, 3);
  Serial.print("\tAz:"); Serial.print(az_ms2, 3);

  Serial.print("\tGx:"); Serial.print(gx_dps, 2);
  Serial.print("\tGy:"); Serial.print(gy_dps, 2);
  Serial.print("\tGz:"); Serial.print(gz_dps, 2);
  
  Serial.print("\tTemp:"); Serial.println(temp_c, 2);

  Serial.println();
  delay(50); // ~20 Hz
}
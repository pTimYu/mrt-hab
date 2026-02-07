#include <Wire.h>
#include "lsm6dsm_minimal.h"

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  Wire.setClock(100000);

  if (!lsm6_init_minimal()) {
    Serial.println("LSM6 init FAILED");
    while (1) delay(1000);
  }

}

void loop() {
  int16_t ax, ay, az, gx, gy, gz, t;

  if (!lsm6_read_raw(ax, ay, az, gx, gy, gz, t)) {
    // Plotter 模式下，出错时最好什么都不打印
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
  Serial.print("Ax(m/s^2)="); Serial.print(ax_ms2, 3);
  Serial.print(" Ay(m/s^2)="); Serial.print(ay_ms2, 3);
  Serial.print(" Az(m/s^2)="); Serial.print(az_ms2, 3);

  Serial.print(" Gx(dps)="); Serial.print(gx_dps, 2);
  Serial.print(" Gy(dps)="); Serial.print(gy_dps, 2);
  Serial.print(" Gz(dps)="); Serial.print(gz_dps, 2);

  Serial.print(" Temp(C)="); Serial.print(temp_c, 2);

  Serial.println();
  delay(50); // ~20 Hz
}
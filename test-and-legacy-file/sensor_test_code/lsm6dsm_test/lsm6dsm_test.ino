#include <Wire.h>
#include "lsm6dsm_sensor.h"
#include "orientation_filter.h"

static const uint8_t LSM6_ADDR = 0x6B;  // LSM I2C Address
IMUData imu_data;
OrientationData revised_data;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LSM test");

  Wire.begin();
  Wire.setClock(100000);

  if (!imu_init()) {
    Serial.println("LSM6 init FAILED");
    while (1) delay(1000);
  }
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz, t;
  

  if (!imu_read(imu_data)) {
    delay(50);
    return;
  }

  float ax_ms2 = imu_data.ax;
  float ay_ms2 = imu_data.ay;
  float az_ms2 = imu_data.az;

  float gx_dps = imu_data.gx;
  float gy_dps = imu_data.gy;
  float gz_dps = imu_data.gz;

  float temp_c = imu_data.temperature;

  revised_data = processIMU(ax_ms2, ay_ms2, az_ms2, gx_dps, gy_dps, 0.01);

  // ===== Serial Plotter format =====
  Serial.print("Ax:"); Serial.print(revised_data.ax, 3);
  Serial.print("\tAy:"); Serial.print(revised_data.ay, 3);
  Serial.print("\tAz:"); Serial.print(revised_data.az, 3);

  Serial.print("\tGx:"); Serial.print(revised_data.gx, 2);
  Serial.print("\tGy:"); Serial.print(revised_data.gy, 2);
  Serial.print("\tGz:"); Serial.print(gz_dps, 2);

  Serial.print("\tRoll:"); Serial.print(revised_data.roll, 2);
  Serial.print("\tPitch:"); Serial.print(revised_data.pitch, 2);
  
  Serial.print("\tTemp:"); Serial.println(temp_c, 2);

  Serial.println();
  delay(50); // ~20 Hz
}
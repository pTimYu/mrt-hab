#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin();              // ESP32 若用自定义引脚：Wire.begin(SDA, SCL);
  Wire.setClock(100000);     // 先用100kHz，最稳

  Serial.println("I2C scan start...");
}

void loop() {
  int nDevices = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Found I2C device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      nDevices++;
    }
  }

  if (nDevices == 0) Serial.println("No I2C devices found.");
  else Serial.println("Scan done.");

  delay(3000);
}

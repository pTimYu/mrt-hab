#include "adafruit_gps.h"

GnssData gps_data;
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for the serial monitor to open
  
  Serial.println("--- Adafruit GPS Test ---");

  if (!gps_init()) {
    Serial.println("ERROR: Adafruit GPS Initialize Failed. Check wiring!");
  } else {
    Serial.println("SUCCESS: GPS Initialized.");
    Serial.println("Waiting for satellite fix...");
  }
}

void loop() {
  gps_read_data(gps_data);

  if (millis() - lastPrintTime > 2000) {
    lastPrintTime = millis();

    Serial.println("\n--- GPS STATUS REPORT ---");
    
    // Check for Fix Status
    if (!gps_data.location_valid) {
        Serial.print("STATUS: [SEARCHING] - Sats in view: ");
        Serial.println(gps_data.satellites);
        Serial.println("Note: Move antenna near a window or outside!");
    } else {
        Serial.print("STATUS: [FIX ACQUIRED] - Sats: ");
        Serial.println(gps_data.satellites);
    }

    // Print Coordinates only if valid
    if (gps_data.location_valid) {
        Serial.print("LAT:  "); Serial.println(gps_data.latitude, 6);
        Serial.print("LONG: "); Serial.println(gps_data.longitude, 6);
        Serial.print("SPD:  "); Serial.print(gps_data.speed_gps); Serial.println(" m/s");
        Serial.print("ALT:  "); Serial.print(gps_data.gps_altitude); Serial.println(" m");
    } else {
        Serial.println("LAT/LONG: (Waiting for lock...)");
    }

    Serial.println("-------------------------");
  }
}
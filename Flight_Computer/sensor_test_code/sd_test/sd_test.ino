// We use SdFat Header file since we need to lower the SPI frequency
#include <SdFat.h>

const int SD_CS_PIN = 9;
// D11(MOSI), D12(MISO), D13(SCK)

SdFat sd;
SdFile myFile;



void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);

  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(4))) { // Slower the SPI frequency to 4MHz
    Serial.println(F("Initialization FAILED!"));
    sd.initErrorPrint(&Serial);
    return;
  }

  // Remove file, no need to add this in flight computer codes
  if (sd.exists("test_num.txt")) {
    Serial.print("test_num.txt ");
    Serial.println("exist");
    if (sd.remove("test_num.txt")) {
      Serial.println("Remove successfully");
    }
  }

  // Open file in the setup
  if (myFile.open("test_num.txt", O_RDWR | O_CREAT | O_AT_END)) {
    myFile.println("SD card testing");
    Serial.println("File open/create successfully");
  }

}

void loop() {
  static int count = 0;
  static uint32_t lastSync = 0;

  count++;
  // Write test, close after each run
  // if (myFile.open("test_num.txt", O_RDWR | O_CREAT | O_AT_END)) {
  //   myFile.print("test: ");
  //   myFile.println(count);
  //   Serial.println(count);
  //   myFile.close(); // Close file // Must execute it for each read/write operation
  // }


  // Using flush() to save data, reduce the cost of close.
  if (!myFile.isOpen()) {
    myFile.open("test_num.txt", O_RDWR | O_CREAT | O_AT_END);
  }

  myFile.print("Test: ");
  myFile.println(count);

  // Save per 2s, reduce the frequent writing
  if (millis() - lastSync > 5000) {
    myFile.flush();
    Serial.println("Data saved");
    lastSync = millis();
  }

  // Recommand to write a close() after landing.

  delay(500);
}
#include <Arduino.h>
#include <SoftwareSerial.h>

#define ROLE_RECEIVER     // ← uncomment on receiver

#define LORA_RX 2
#define LORA_TX 7   
SoftwareSerial LoRa(LORA_RX, LORA_TX);

char buf[256];

/* ---------- UTILITY: ASCII STRING TO HEX ---------- */
// Converts ASCII string to hex representation
// Example: "23.5" -> "32332E35"
void stringToHex(const char* input, char* output) {
  int j = 0;
  for (int i = 0; input[i] != '\0'; i++) {
    sprintf(&output[j], "%02X", (unsigned char)input[i]);
    j += 2;
  }
  output[j] = '\0';
}

/* ---------- UTILITY: HEX TO ASCII STRING ---------- */
// Converts hex representation back to ASCII string
// Example: "32332E35" -> "23.5"
void hexToString(const char* input, char* output) {
  int len = strlen(input);
  int j = 0;
  for (int i = 0; i < len; i += 2) {
    char hexByte[3] = {input[i], input[i+1], '\0'};
    output[j++] = (char)strtol(hexByte, NULL, 16);
  }
  output[j] = '\0';
}

/* ---------- SEND AT COMMAND ---------- */
void sendAT(const char *cmd) {
  LoRa.print(cmd);
  Serial.print(cmd);
}

bool waitFor(const char *keyword, unsigned long timeout) {
  unsigned long start = millis();
  int i = 0;
  memset(buf, 0, sizeof(buf));
  
  while (millis() - start < timeout) {
    while (LoRa.available()) {
      if (i < 255) {
        buf[i++] = LoRa.read();
        Serial.print(buf[i-1]);
        delay(2);
      } else {
        LoRa.read();  // discard overflow
      }
    }
    if (strstr(buf, keyword)) return true;
  }
  return false;
}

/* ---------- INITIALIZE LORA ---------- */
bool initLoRa() {
  sendAT("AT\r\n");
  if (!waitFor("+AT: OK", 500)) return false;
  
  sendAT("AT+MODE=TEST\r\n");
  waitFor("+MODE: TEST", 500);
  
  sendAT("AT+TEST=RFCFG,866,SF12,125,12,15,14,ON,OFF,OFF\r\n");
  waitFor("+TEST: RFCFG", 500);
  
  return true;
}

/* ---------- SEND STRING DATA ---------- */
void sendString(const char* data) {
  char hexPayload[128];
  char cmd[256];
  
  // Convert ASCII string to hex
  stringToHex(data, hexPayload);
  
  // Build AT command
  sprintf(cmd, "AT+TEST=TXLRPKT,\"%s\"\r\n", hexPayload);
  
  Serial.print("Sending: ");
  Serial.println(data);
  
  sendAT(cmd);
  
  if (waitFor("TX DONE", 2000)) {
    Serial.println("✓ Sent successfully\n");
  } else {
    Serial.println("✗ Send failed\n");
  }
}

/* ---------- RECEIVE STRING DATA ---------- */
void receiveString() {
  sendAT("AT+TEST=RXLRPKT\r\n");
  
  if (waitFor("+TEST: RXLRPKT", 1000)) {
    Serial.println("RX mode active, waiting for packet...");
  }
  
  if (waitFor("+TEST: RX", 5000)) {
    Serial.println("\n✓ Packet received!");
    Serial.println("Raw response:");
    Serial.println(buf);
    
    // Extract hex payload between quotes
    char* start = strchr(buf, '"');
    if (start) {
      start++;  // move past opening quote
      char* end = strchr(start, '"');
      if (end) {
        *end = '\0';  // null-terminate at closing quote
        
        // Convert hex back to ASCII string
        char decodedString[128];
        hexToString(start, decodedString);
        
        Serial.print("\nDecoded string: ");
        Serial.println(decodedString);
        Serial.println("-------------------\n");
      }
    }
  } else {
    Serial.println("✗ No packet received (timeout)\n");
  }
}

/* ---------- SETUP ---------- */
void setup() {
  Serial.begin(115200);
  LoRa.begin(9600);
  
  delay(1000);
  Serial.println("\n=== LoRa String Communication ===\n");
  
  if (!initLoRa()) {
    Serial.println("LoRa module not found!");
    while (1);
  }
  
#ifdef ROLE_RECEIVER
  Serial.println("Mode: RECEIVER\n");
#else
  Serial.println("Mode: TRANSMITTER\n");
#endif
}

/* ---------- LOOP ---------- */
void loop() {
#ifdef ROLE_RECEIVER
  receiveString();
  
#else
  // Example: sending different types of string data
  static int counter = 0;
  
  switch(counter % 4) {
    case 0:
      sendString("23.5");      // float as string
      break;
    case 1:
      sendString("42");        // int as string
      break;
    case 2:
      sendString("OK");        // status message
      break;
    case 3:
      sendString("Temp:25.3"); // formatted data
      break;
  }
  
  counter++;
  delay(5000);  // send every 5 seconds
#endif
}

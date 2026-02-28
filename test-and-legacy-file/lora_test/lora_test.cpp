#include <Arduino.h>
#include <SoftwareSerial.h>

#define MAX_DIGITS 120
#define MAX_HEX_PAYLOAD (MAX_DIGITS * 2)

#define ROLE_RECEIVER     // ← uncomment on receiver

#define LORA_RX 2
#define LORA_TX 7   
SoftwareSerial LoRa(LORA_RX, LORA_TX);

char buf[384];

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
      if (i < (int)(sizeof(buf) - 1)) {
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

/* ---------- DIGIT ARRAY <-> STRING ---------- */
bool digitArrayToString(const uint8_t* digits, size_t len, char* out, size_t outSize) {
  if (len + 1 > outSize) return false;

  for (size_t i = 0; i < len; i++) {
    if (digits[i] > 9) return false;
    out[i] = (char)('0' + digits[i]);
  }

  out[len] = '\0';
  return true;
}

bool stringToDigitArray(const char* input, uint8_t* outDigits, size_t maxDigits, size_t& outLen) {
  size_t len = strlen(input);
  if (len > maxDigits) return false;

  for (size_t i = 0; i < len; i++) {
    if (input[i] < '0' || input[i] > '9') return false;
    outDigits[i] = (uint8_t)(input[i] - '0');
  }

  outLen = len;
  return true;
}

/* ---------- SEND DIGIT ARRAY ---------- */
bool sendDigitArray(const uint8_t* digits, size_t len) {
  char payload[MAX_DIGITS + 1];
  char hexPayload[MAX_HEX_PAYLOAD + 1];
  char cmd[512];

  if (!digitArrayToString(digits, len, payload, sizeof(payload))) {
    Serial.println("✗ Invalid digit array (size/value)");
    return false;
  }

  stringToHex(payload, hexPayload);
  
  // Build AT command
  sprintf(cmd, "AT+TEST=TXLRPKT,\"%s\"\r\n", hexPayload);
  
  Serial.print("Sending digits: ");
  Serial.println(payload);
  
  sendAT(cmd);
  
  if (waitFor("TX DONE", 2000)) {
    Serial.println("✓ Sent successfully\n");
    return true;
  } else {
    Serial.println("✗ Send failed\n");
    return false;
  }
}

/* ---------- RECEIVE DIGIT ARRAY ---------- */
bool receiveDigitArray(uint8_t* outDigits, size_t maxDigits, size_t& outLen) {
  outLen = 0;
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
        char decodedString[MAX_DIGITS + 1];
        hexToString(start, decodedString);

        if (!stringToDigitArray(decodedString, outDigits, maxDigits, outLen)) {
          Serial.println("✗ Invalid payload format (non-digit or too long)\n");
          return false;
        }

        Serial.print("\nDecoded digits (len=");
        Serial.print(outLen);
        Serial.println("):");
        for (size_t i = 0; i < outLen; i++) {
          Serial.print(outDigits[i]);
          if (i + 1 < outLen) Serial.print(',');
        }
        Serial.println("\n-------------------\n");
        return true;
      }
    }
    Serial.println("✗ Could not parse RX payload\n");
    return false;
  } else {
    Serial.println("✗ No packet received (timeout)\n");
    return false;
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
  uint8_t rxDigits[MAX_DIGITS];
  size_t rxLen = 0;
  receiveDigitArray(rxDigits, MAX_DIGITS, rxLen);
  
#else
  const uint8_t txDigits[] = {
    0,1,4,8,2,0,2,0,4,8,2,9,4,9,8,3,8,4,
    1,0,5,7,2,3,6,1,5,1,2,9,7,0,4,3,8,5
  };
  sendDigitArray(txDigits, sizeof(txDigits) / sizeof(txDigits[0]));
  delay(5000);  // send every 5 seconds
#endif
}

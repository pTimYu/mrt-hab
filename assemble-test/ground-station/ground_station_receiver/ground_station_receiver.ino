// ── HAB Ground Station Receiver ───────────────────────────────────────────────
// Target board: Arduino UNO (ATmega328P)
//
// Hardware Serial (pins 0/1) → USB to PC
// SoftwareSerial             → LoRa module (AT commands, 9600 baud)
//
// Receives 37-byte binary LoRa telemetry packets from the flight computer,
// unpacks the data, and outputs it over hardware Serial (USB).
//
// Output mode controlled by DEBUG_MODE:
//   DEBUG_MODE 1 → human-readable text (for Arduino Serial Monitor)
//   DEBUG_MODE 0 → raw 25-byte binary  (for simple-terminal-visualization)
//
// Wiring:
//   LoRa TX  → UNO pin 10 (SOFT_RX_PIN)
//   LoRa RX  → UNO pin 11 (SOFT_TX_PIN)
//   LoRa VCC → 3.3V,  GND → GND
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <SoftwareSerial.h>

// ── Debug switch ─────────────────────────────────────────────────────────────
// Set to 1 for human-readable Serial Monitor output (development/testing).
// Set to 0 for clean 25-byte binary output (flight / visualization).
#define DEBUG_MODE  1

// ── Pin config ───────────────────────────────────────────────────────────────
#define SOFT_RX_PIN     10
#define SOFT_TX_PIN     11

// ── Radio config (must match flight computer) ────────────────────────────────
#define LORA_BAUD       9600
#define LORA_RF_CONFIG  "866,SF12,125,12,15,14,ON,OFF,OFF"

// ── USB Serial baud ──────────────────────────────────────────────────────────
#define SERIAL_BAUD     115200

// ── SoftwareSerial instance for LoRa ─────────────────────────────────────────
SoftwareSerial loraSerial(SOFT_RX_PIN, SOFT_TX_PIN);  // RX, TX

// ── Telemetry packet layout (from flight computer) ───────────────────────────
// 37 bytes, little-endian:
//   [ 0.. 3]  time               (int32)     — flight elapsed ms
//   [ 4.. 7]  altitude           (float)
//   [ 8..11]  vertical_velocity  (float)
//   [12..15]  temperature        (float)
//   [16..19]  pressure           (float)
//   [20..23]  ax                 (float)
//   [24..27]  ay                 (float)
//   [28..31]  az                 (float)
//   [32..35]  voltage            (float)
//   [36]      flags              (uint8_t)
static const size_t TELEM_PACKET_SIZE = 37;

static const int OUTPUT_BYTES   = 25;
static const int OUTPUT_DIGITS  = 60;

// ── Struct declared early so Arduino IDE auto-prototypes resolve correctly ───
struct TelemetryData {
    int32_t time_ms;
    float   altitude;
    float   vertical_velocity;
    float   temperature;
    float   pressure;
    float   ax, ay, az;
    float   voltage;
    uint8_t flags;
};

// ── AT command helpers ───────────────────────────────────────────────────────
static char s_buf[384];

static void send_at(const char* cmd) {
    loraSerial.print(cmd);
}

static bool wait_for(const char* keyword, unsigned long timeout) {
    unsigned long start = millis();
    int idx = 0;
    memset(s_buf, 0, sizeof(s_buf));

    while (millis() - start < timeout) {
        while (loraSerial.available()) {
            if (idx < (int)(sizeof(s_buf) - 1)) {
                s_buf[idx++] = (char)loraSerial.read();
                delay(2);
            } else {
                loraSerial.read();  // discard overflow
            }
        }
        if (strstr(s_buf, keyword)) return true;
    }
    return false;
}

// ── Hex decode ───────────────────────────────────────────────────────────────
static size_t hex_to_bytes(const char* hex, uint8_t* out, size_t max_len) {
    size_t hex_len = strlen(hex);
    if (hex_len % 2 != 0) return 0;
    size_t n = hex_len / 2;
    if (n > max_len) return 0;

    for (size_t i = 0; i < n; i++) {
        char pair[3] = { hex[i * 2], hex[i * 2 + 1], '\0' };
        out[i] = (uint8_t)strtol(pair, NULL, 16);
    }
    return n;
}

// ── Unpack telemetry from binary buffer ──────────────────────────────────────
static void unpack_telemetry(const uint8_t* buf, TelemetryData& t) {
    size_t pos = 0;
    memcpy(&t.time_ms,           &buf[pos], 4); pos += 4;
    memcpy(&t.altitude,          &buf[pos], 4); pos += 4;
    memcpy(&t.vertical_velocity, &buf[pos], 4); pos += 4;
    memcpy(&t.temperature,       &buf[pos], 4); pos += 4;
    memcpy(&t.pressure,          &buf[pos], 4); pos += 4;
    memcpy(&t.ax,                &buf[pos], 4); pos += 4;
    memcpy(&t.ay,                &buf[pos], 4); pos += 4;
    memcpy(&t.az,                &buf[pos], 4); pos += 4;
    memcpy(&t.voltage,           &buf[pos], 4); pos += 4;
    t.flags = buf[pos];
}

// ── Big-number arithmetic: 60-digit decimal → 25-byte big-endian ─────────────
static uint8_t divmod256(uint8_t* digits, int len) {
    uint16_t remainder = 0;
    for (int i = 0; i < len; i++) {
        uint16_t val = remainder * 10 + digits[i];
        digits[i] = (uint8_t)(val / 256);
        remainder  = val % 256;
    }
    return (uint8_t)remainder;
}

static void decimal_to_bytes(const char* decimal, uint8_t* out, int out_len) {
    uint8_t digs[60];
    for (int i = 0; i < OUTPUT_DIGITS; i++) {
        digs[i] = decimal[i] - '0';
    }

    uint8_t tmp[25];
    for (int b = 0; b < out_len; b++) {
        tmp[b] = divmod256(digs, OUTPUT_DIGITS);
    }

    for (int i = 0; i < out_len; i++) {
        out[i] = tmp[out_len - 1 - i];
    }
}

// ── Helper: clamp ────────────────────────────────────────────────────────────
static int32_t clamp_i(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ── Encode telemetry → 60-digit decimal string ──────────────────────────────
static void encode_to_decimal(const TelemetryData& t, char* out60) {
    int pos = 0;

    // time (4 digits): elapsed ms → MMSS
    uint32_t total_sec = (uint32_t)(t.time_ms / 1000);
    uint32_t mm = (total_sec / 60) % 100;
    uint32_t ss = total_sec % 60;
    int32_t time_val = clamp_i((int32_t)(mm * 100 + ss), 0, 9999);
    sprintf(&out60[pos], "%04ld", (long)time_val);
    pos += 4;

    // latitude (9 digits): placeholder 0
    sprintf(&out60[pos], "%09ld", 0L);
    pos += 9;

    // longitude (9 digits): placeholder 0
    sprintf(&out60[pos], "%09ld", 0L);
    pos += 9;

    // speed_gps (3 digits): placeholder 0
    sprintf(&out60[pos], "%03ld", 0L);
    pos += 3;

    // heading_gps (3 digits): placeholder 0
    sprintf(&out60[pos], "%03ld", 0L);
    pos += 3;

    // altitude (5 digits): metres integer
    int32_t alt_enc = clamp_i((int32_t)t.altitude, 0, 99999);
    sprintf(&out60[pos], "%05ld", (long)alt_enc);
    pos += 5;

    // voltage (3 digits): V × 100
    int32_t volt_enc = clamp_i((int32_t)(t.voltage * 100.0f), 0, 999);
    sprintf(&out60[pos], "%03ld", (long)volt_enc);
    pos += 3;

    // RSSI (3 digits): 0
    sprintf(&out60[pos], "%03d", 0);
    pos += 3;

    // Gain (3 digits): 0
    sprintf(&out60[pos], "%03d", 0);
    pos += 3;

    // accel_x (6 digits): (m/s² + 100) × 1000
    int32_t ax_enc = clamp_i((int32_t)((t.ax + 100.0f) * 1000.0f), 0, 199999);
    sprintf(&out60[pos], "%06ld", (long)ax_enc);
    pos += 6;

    // accel_y (6 digits)
    int32_t ay_enc = clamp_i((int32_t)((t.ay + 100.0f) * 1000.0f), 0, 199999);
    sprintf(&out60[pos], "%06ld", (long)ay_enc);
    pos += 6;

    // accel_z (6 digits)
    int32_t az_enc = clamp_i((int32_t)((t.az + 100.0f) * 1000.0f), 0, 199999);
    sprintf(&out60[pos], "%06ld", (long)az_enc);
    pos += 6;

    out60[OUTPUT_DIGITS] = '\0';
}

// ── LoRa init ────────────────────────────────────────────────────────────────
static bool lora_init_ground() {
    loraSerial.begin(LORA_BAUD);

    send_at("AT\r\n");
    if (!wait_for("+AT: OK", 500)) return false;

    send_at("AT+MODE=TEST\r\n");
    wait_for("+MODE: TEST", 500);

    char cfg_cmd[128];
    snprintf(cfg_cmd, sizeof(cfg_cmd),
             "AT+TEST=RFCFG,%s\r\n", LORA_RF_CONFIG);
    send_at(cfg_cmd);
    wait_for("+TEST: RFCFG", 500);

    return true;
}

// ── Receive one LoRa packet ──────────────────────────────────────────────────
static size_t lora_receive_packet(uint8_t* out_data, size_t max_len,
                                   unsigned long rx_timeout_ms) {
    send_at("AT+TEST=RXLRPKT\r\n");
    wait_for("+TEST: RXLRPKT", 1000);

    if (!wait_for("+TEST: RX", rx_timeout_ms)) {
        return 0;
    }

    char* start = strchr(s_buf, '"');
    if (!start) return 0;
    start++;

    char* end = strchr(start, '"');
    if (!end) return 0;
    *end = '\0';

    return hex_to_bytes(start, out_data, max_len);
}

// ═════════════════════════════════════════════════════════════════════════════
// setup() / loop()
// ═════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(SERIAL_BAUD);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

#if DEBUG_MODE
    Serial.println(F("[GND] HAB Ground Station Receiver (UNO)"));
    Serial.println(F("[GND] *** DEBUG MODE — human-readable output ***"));
    Serial.println(F("[GND] Initialising LoRa on SoftwareSerial..."));
#endif

    if (!lora_init_ground()) {
#if DEBUG_MODE
        Serial.println(F("[GND] ERROR: LoRa init failed!"));
#endif
        while (true) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(200);
            digitalWrite(LED_BUILTIN, LOW);
            delay(200);
        }
    }

#if DEBUG_MODE
    Serial.println(F("[GND] LoRa ready. Listening..."));
#endif

    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    // 1. Receive LoRa packet (blocks up to 10s)
    uint8_t rx_buf[64];
    size_t  rx_len = lora_receive_packet(rx_buf, sizeof(rx_buf), 10000);

    if (rx_len < TELEM_PACKET_SIZE) {
#if DEBUG_MODE
        Serial.println(F("[GND] RX timeout or short packet"));
#endif
        return;
    }

    // LED blink to indicate packet received
    digitalWrite(LED_BUILTIN, LOW);

    // 2. Unpack binary telemetry
    TelemetryData telem;
    unpack_telemetry(rx_buf, telem);

#if DEBUG_MODE
    // ── DEBUG MODE: human-readable output ─────────────────────────────────
    Serial.println(F("──────────────────────────────"));
    Serial.print(F("[GND] Time: "));    Serial.print(telem.time_ms);
    Serial.print(F(" ms  Alt: "));      Serial.print(telem.altitude, 1);
    Serial.print(F(" m  VVel: "));      Serial.print(telem.vertical_velocity, 2);
    Serial.println(F(" m/s"));

    Serial.print(F("[GND] Temp: "));    Serial.print(telem.temperature, 1);
    Serial.print(F(" C  Pres: "));      Serial.print(telem.pressure, 1);
    Serial.print(F(" hPa  Volt: "));    Serial.print(telem.voltage, 2);
    Serial.println(F(" V"));

    Serial.print(F("[GND] Accel: "));
    Serial.print(telem.ax, 2); Serial.print(F(", "));
    Serial.print(telem.ay, 2); Serial.print(F(", "));
    Serial.print(telem.az, 2); Serial.println(F(" m/s2"));

    Serial.print(F("[GND] Flags: BMP="));
    Serial.print((telem.flags & 0x01) ? F("OK") : F("FAIL"));
    Serial.print(F("  IMU="));
    Serial.print((telem.flags & 0x02) ? F("OK") : F("FAIL"));
    Serial.print(F("  GNSS="));
    Serial.println((telem.flags & 0x04) ? F("OK") : F("FAIL"));

#else
    // ── FLIGHT MODE: 25-byte binary for simple-terminal-visualization ─────
    char decimal[OUTPUT_DIGITS + 1];
    encode_to_decimal(telem, decimal);

    uint8_t output[OUTPUT_BYTES];
    decimal_to_bytes(decimal, output, OUTPUT_BYTES);

    Serial.write(output, OUTPUT_BYTES);
    Serial.flush();
#endif

    digitalWrite(LED_BUILTIN, HIGH);
}

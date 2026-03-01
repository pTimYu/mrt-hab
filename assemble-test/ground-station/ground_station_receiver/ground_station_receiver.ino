// ── HAB Ground Station Receiver ───────────────────────────────────────────────
// Target board: Arduino UNO (ATmega328P)
//
// Hardware Serial (pins 0/1) → USB to PC
// SoftwareSerial             → LoRa module (AT commands, 9600 baud)
//
// Receives 23-byte LoRa telemetry packets (54-digit decimal, no RSSI/SNR),
// measures RSSI and SNR locally from the LoRa module's AT response,
// inserts them to form a 60-digit decimal (25-byte binary) output to the PC.
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

// ── Packet sizes ─────────────────────────────────────────────────────────────
//
// LoRa packet (from flight computer):
//   54 decimal digits packed into 23 bytes (big-endian).
//   ceil(54 × log₂10 / 8) = ceil(179.38 / 8) = 23
//
// PC output (to simple-terminal-visualization):
//   60 decimal digits packed into 25 bytes (big-endian).
//   The extra 6 digits are RSSI (3) + SNR (3) inserted by this ground station.
//
static const int TELEM_DIGITS      = 54;
static const int TELEM_PACKET_SIZE = 23;
static const int OUTPUT_DIGITS     = 60;
static const int OUTPUT_BYTES      = 25;

// ── Decoded telemetry (parsed from the 54-digit string) ─────────────────────
struct TelemetryData {
    int32_t time_hhmmss;        // HHMMSS
    float   latitude;           // degrees
    float   longitude;          // degrees
    float   speed_gps;          // m/s
    int     heading_gps;        // degrees
    float   altitude;           // m (Kalman)
    float   voltage;            // V
    float   ax, ay, az;         // m/s²
};

// ── Ground-station radio quality (measured locally) ──────────────────────────
struct RadioQuality {
    int rssi;       // dBm (negative, e.g. -80)
    int snr;        // dB  (e.g. 10)
};

// ── AT command helpers ───────────────────────────────────────────────────────
static char s_buf[384];
static int  s_buf_len = 0;

static void send_at(const char* cmd) {
    loraSerial.print(cmd);
}

static bool wait_for(const char* keyword, unsigned long timeout) {
    unsigned long start = millis();
    s_buf_len = 0;
    memset(s_buf, 0, sizeof(s_buf));

    while (millis() - start < timeout) {
        while (loraSerial.available()) {
            if (s_buf_len < (int)(sizeof(s_buf) - 1)) {
                s_buf[s_buf_len++] = (char)loraSerial.read();
                delay(2);
            } else {
                loraSerial.read();
            }
        }
        if (strstr(s_buf, keyword)) return true;
    }
    return false;
}

static void drain_extra(unsigned long extra_ms) {
    unsigned long start = millis();
    while (millis() - start < extra_ms) {
        while (loraSerial.available()) {
            if (s_buf_len < (int)(sizeof(s_buf) - 1)) {
                s_buf[s_buf_len++] = (char)loraSerial.read();
                delay(2);
            } else {
                loraSerial.read();
            }
        }
    }
    s_buf[s_buf_len] = '\0';
}

// ── RSSI / SNR parser ────────────────────────────────────────────────────────
// Wio-E5 response:  +TEST: RSSI:-80 SNR:10   (or comma-separated variant)
static void parse_rssi_snr(RadioQuality& rq) {
    rq.rssi = 0;
    rq.snr  = 0;

    const char* p = strstr(s_buf, "RSSI:");
    if (p) { p += 5; rq.rssi = atoi(p); }

    p = strstr(s_buf, "SNR:");
    if (p) { p += 4; rq.snr = atoi(p); }
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

// ── Bytes → decimal string (big-endian bytes → zero-padded decimal) ──────────
// For each byte (MSB first): accumulator = accumulator × 256 + byte
// Accumulator is stored as a fixed-width array of decimal digits.
static void bytes_to_decimal(const uint8_t* in, int n_bytes,
                             char* out, int n_digits) {
    uint8_t digs[60];
    memset(digs, 0, sizeof(digs));

    for (int b = 0; b < n_bytes; b++) {
        // Multiply accumulator by 256 and add in[b]
        uint16_t carry = in[b];
        for (int d = n_digits - 1; d >= 0; d--) {
            uint16_t val = (uint16_t)digs[d] * 256 + carry;
            digs[d] = val % 10;
            carry   = val / 10;
        }
        // Overflow carry is silently dropped (should not happen with valid data)
    }

    for (int i = 0; i < n_digits; i++) {
        out[i] = '0' + digs[i];
    }
    out[n_digits] = '\0';
}

// ── Decimal string → bytes (big-endian, repeated /256) ───────────────────────
static uint8_t divmod256(uint8_t* digits, int len) {
    uint16_t remainder = 0;
    for (int i = 0; i < len; i++) {
        uint16_t val = remainder * 10 + digits[i];
        digits[i] = (uint8_t)(val / 256);
        remainder  = val % 256;
    }
    return (uint8_t)remainder;
}

static void decimal_to_bytes(const char* decimal, int n_digits,
                             uint8_t* out, int n_bytes) {
    uint8_t digs[60];
    for (int i = 0; i < n_digits; i++) {
        digs[i] = decimal[i] - '0';
    }

    uint8_t tmp[25];
    for (int b = 0; b < n_bytes; b++) {
        tmp[b] = divmod256(digs, n_digits);
    }

    for (int i = 0; i < n_bytes; i++) {
        out[i] = tmp[n_bytes - 1 - i];
    }
}

// ── Helper: clamp ────────────────────────────────────────────────────────────
static int32_t clamp_i(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ── Parse 54-digit decimal → TelemetryData ───────────────────────────────────
//
// LoRa digit layout (54 total):
//   [ 0.. 5]  Time       (6)  HHMMSS
//   [ 6..14]  Latitude   (9)  (lat × 1e5) + 1e8
//   [15..23]  Longitude  (9)  (lon × 1e5) + 1e8
//   [24..26]  Speed      (3)  speed × 10
//   [27..29]  Heading    (3)  heading
//   [30..35]  Height     (6)  altitude × 10
//   [36..38]  Voltage    (3)  voltage × 100
//   [39..43]  Accel X    (5)  (ax × 100) + 10000
//   [44..48]  Accel Y    (5)  (ay × 100) + 10000
//   [49..53]  Accel Z    (5)  (az × 100) + 10000
//
static void parse_telemetry(const char* dec54, TelemetryData& t) {
    // Helper: extract N chars from `dec54` at offset `pos`, return as long
    auto getInt = [&](int pos, int len) -> int32_t {
        char tmp[10];
        memcpy(tmp, &dec54[pos], len);
        tmp[len] = '\0';
        return atol(tmp);
    };

    t.time_hhmmss = getInt(0, 6);
    t.latitude    = (getInt(6, 9) - 100000000L) / 100000.0f;
    t.longitude   = (getInt(15, 9) - 100000000L) / 100000.0f;
    t.speed_gps   = getInt(24, 3) / 10.0f;
    t.heading_gps = (int)getInt(27, 3);
    t.altitude    = getInt(30, 6) / 10.0f;
    t.voltage     = getInt(36, 3) / 100.0f;
    t.ax          = (getInt(39, 5) - 10000) / 100.0f;
    t.ay          = (getInt(44, 5) - 10000) / 100.0f;
    t.az          = (getInt(49, 5) - 10000) / 100.0f;
}

// ── Build 60-digit PC output string ──────────────────────────────────────────
//
// PC digit layout (60 total):
//   [ 0.. 5]  Time       (6)  — pass through from LoRa
//   [ 6..14]  Latitude   (9)  — pass through
//   [15..23]  Longitude  (9)  — pass through
//   [24..26]  Speed      (3)  — pass through
//   [27..29]  Heading    (3)  — pass through
//   [30..35]  Height     (6)  — pass through
//   [36..38]  Voltage    (3)  — pass through
//   [39..41]  RSSI       (3)  — |RSSI| in dBm (inserted by GS)
//   [42..44]  SNR/Gain   (3)  — (SNR × 10) + 200  (inserted by GS)
//   [45..49]  Accel X    (5)  — pass through from LoRa digits [39..43]
//   [50..54]  Accel Y    (5)  — pass through from LoRa digits [44..48]
//   [55..59]  Accel Z    (5)  — pass through from LoRa digits [49..53]
//
static void build_pc_output(const char* dec54, const RadioQuality& rq,
                            char* out60) {
    // Copy first 39 digits (Time through Voltage) verbatim
    memcpy(&out60[0], &dec54[0], 39);

    // Insert RSSI (3 digits): |RSSI| in dBm
    int32_t rssi_enc = clamp_i((int32_t)abs(rq.rssi), 0, 999);
    sprintf(&out60[39], "%03ld", (long)rssi_enc);

    // Insert SNR (3 digits): (SNR × 10) + 200
    int32_t snr_enc = clamp_i((int32_t)(rq.snr * 10 + 200), 0, 999);
    sprintf(&out60[42], "%03ld", (long)snr_enc);

    // Copy last 15 digits (Accel X/Y/Z) from LoRa positions [39..53]
    memcpy(&out60[45], &dec54[39], 15);

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

// ── Receive one LoRa packet + radio quality ──────────────────────────────────
static size_t lora_receive_packet(uint8_t* out_data, size_t max_len,
                                   RadioQuality& rq,
                                   unsigned long rx_timeout_ms) {
    rq.rssi = 0;
    rq.snr  = 0;

    send_at("AT+TEST=RXLRPKT\r\n");
    wait_for("+TEST: RXLRPKT", 1000);

    if (!wait_for("+TEST: RX", rx_timeout_ms)) {
        return 0;
    }

    // Capture trailing RSSI/SNR line
    drain_extra(300);
    parse_rssi_snr(rq);

    // Extract hex payload
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
    // 1. Receive LoRa packet + radio quality
    uint8_t      rx_buf[128];
    RadioQuality rq;
    size_t       rx_len = lora_receive_packet(rx_buf, sizeof(rx_buf), rq, 10000);

    if (rx_len < (size_t)TELEM_PACKET_SIZE) {
#if DEBUG_MODE
        Serial.println(F("[GND] RX timeout or short packet"));
#endif
        return;
    }

    digitalWrite(LED_BUILTIN, LOW);     // blink on packet

    // 2. Decode 23 bytes → 54-digit decimal string
    char dec54[TELEM_DIGITS + 1];
    bytes_to_decimal(rx_buf, TELEM_PACKET_SIZE, dec54, TELEM_DIGITS);

#if DEBUG_MODE
    // 3a. DEBUG: parse and print human-readable
    TelemetryData telem;
    parse_telemetry(dec54, telem);

    Serial.println(F("──────────────────────────────"));
    Serial.print(F("[GND] GNSS Time: "));  Serial.print(telem.time_hhmmss);
    Serial.print(F("  Alt: "));            Serial.print(telem.altitude, 1);
    Serial.println(F(" m"));

    Serial.print(F("[GND] GPS: "));
    Serial.print(telem.latitude, 5);       Serial.print(F(", "));
    Serial.print(telem.longitude, 5);      Serial.print(F("  Spd: "));
    Serial.print(telem.speed_gps, 1);      Serial.print(F(" m/s  Hdg: "));
    Serial.println(telem.heading_gps);

    Serial.print(F("[GND] Volt: "));       Serial.print(telem.voltage, 2);
    Serial.println(F(" V"));

    Serial.print(F("[GND] Accel: "));
    Serial.print(telem.ax, 2); Serial.print(F(", "));
    Serial.print(telem.ay, 2); Serial.print(F(", "));
    Serial.print(telem.az, 2); Serial.println(F(" m/s2"));

    Serial.print(F("[GND] Radio: RSSI="));
    Serial.print(rq.rssi);
    Serial.print(F(" dBm  SNR="));
    Serial.print(rq.snr);
    Serial.println(F(" dB"));

    Serial.print(F("[GND] Raw 54: "));
    Serial.println(dec54);

#else
    // 3b. FLIGHT: build 60-digit string with RSSI/SNR → 25-byte binary → PC
    char dec60[OUTPUT_DIGITS + 1];
    build_pc_output(dec54, rq, dec60);

    uint8_t output[OUTPUT_BYTES];
    decimal_to_bytes(dec60, OUTPUT_DIGITS, output, OUTPUT_BYTES);

    Serial.write(output, OUTPUT_BYTES);
    Serial.flush();
#endif

    digitalWrite(LED_BUILTIN, HIGH);
}

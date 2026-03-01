// ── HAB Ground Station Receiver ───────────────────────────────────────────────
// Target board: Arduino UNO (ATmega328P)
//
// Hardware Serial (pins 0/1) → USB to PC
// SoftwareSerial             → LoRa module (AT commands, 9600 baud)
//
// Receives 53-byte binary LoRa telemetry packets from the flight computer,
// unpacks the data, and outputs it over hardware Serial (USB).
//
// RSSI and SNR are measured locally by the ground station's LoRa module
// on each received packet.  They are encoded into the 60-digit output
// for the simple-terminal-visualization.
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

// ── Telemetry packet layout (from flight computer) ───────────────────────────
// 53 bytes, little-endian:
//   [ 0.. 3]  time               (int32)     — flight elapsed ms
//   [ 4.. 7]  altitude           (float)     — Kalman filtered
//   [ 8..11]  vertical_velocity  (float)
//   [12..15]  temperature        (float)
//   [16..19]  pressure           (float)
//   [20..23]  ax                 (float)
//   [24..27]  ay                 (float)
//   [28..31]  az                 (float)
//   [32..35]  voltage            (float)
//   [36..37]  time_now           (int16)     — GNSS time MMSS
//   [38..41]  latitude           (float)
//   [42..45]  longitude          (float)
//   [46..49]  speed_gps          (float)
//   [50..51]  heading_gps        (int16)
//   [52]      flags              (uint8_t)
static const size_t TELEM_PACKET_SIZE = 53;

static const int OUTPUT_BYTES   = 25;
static const int OUTPUT_DIGITS  = 60;

// ── Struct ───────────────────────────────────────────────────────────────────
struct TelemetryData {
    int32_t time_ms;
    float   altitude;
    float   vertical_velocity;
    float   temperature;
    float   pressure;
    float   ax, ay, az;
    float   voltage;
    // GPS
    int16_t time_now;           // GNSS time as MMSS
    float   latitude;
    float   longitude;
    float   speed_gps;          // m/s
    int16_t heading_gps;        // degrees
    // Flags
    uint8_t flags;
};

// ── Ground-station radio quality (measured locally, NOT from FC packet) ──────
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

// Block until `keyword` appears in the module response or `timeout` expires.
// The full response is captured in s_buf for later parsing.
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
                loraSerial.read();  // discard overflow
            }
        }
        if (strstr(s_buf, keyword)) return true;
    }
    return false;
}

// Continue reading bytes into s_buf (without clearing) for up to `extra_ms`.
// The Wio-E5 sends the RSSI/SNR line shortly after the RX payload line.
// This gives the module time to finish transmitting those trailing fields.
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
// Wio-E5 response after receiving a packet:
//   +TEST: RX "<hex>"
//   +TEST: RSSI:-80 SNR:10
//
// Some firmware versions use commas:
//   +TEST: RX "<hex>", RSSI:-80, SNR:10
//
// Scans s_buf for "RSSI:" and "SNR:" and extracts the integer values.
static void parse_rssi_snr(RadioQuality& rq) {
    rq.rssi = 0;
    rq.snr  = 0;

    const char* p = strstr(s_buf, "RSSI:");
    if (p) {
        p += 5;               // skip "RSSI:"
        rq.rssi = atoi(p);   // handles leading minus sign
    }

    p = strstr(s_buf, "SNR:");
    if (p) {
        p += 4;               // skip "SNR:"
        rq.snr = atoi(p);
    }
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
    // GPS fields
    memcpy(&t.time_now,          &buf[pos], 2); pos += 2;
    memcpy(&t.latitude,          &buf[pos], 4); pos += 4;
    memcpy(&t.longitude,         &buf[pos], 4); pos += 4;
    memcpy(&t.speed_gps,         &buf[pos], 4); pos += 4;
    memcpy(&t.heading_gps,       &buf[pos], 2); pos += 2;
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

// ── Encode telemetry + radio quality → 60-digit decimal string ──────────────
// Format matches ground-station visualization expectations:
//   Time(4) + Lat(9) + Lon(9) + Speed(3) + Heading(3) +
//   Alt(5) + Voltage(3) + RSSI(3) + Gain(3) + AccX(6) + AccY(6) + AccZ(6)
//   = 60 digits total
//
// RSSI encoding  (3 digits, 000–999):
//   Stored as |RSSI|.  e.g. -80 dBm → 080.
//   Visualization decodes: data.RSSI = stof(getNext(3))  → 80
//
// Gain / SNR encoding  (3 digits, 000–999):
//   Stored as SNR × 10.  e.g. 12 dB → 120.
//   Visualization decodes: data.Gain = stof(getNext(3)) / 10.0 → 12.0
//
static void encode_to_decimal(const TelemetryData& t,
                              const RadioQuality& rq,
                              char* out60) {
    int pos = 0;

    // Time (4 digits): GNSS MMSS directly
    int32_t time_val = clamp_i((int32_t)t.time_now, 0, 9999);
    sprintf(&out60[pos], "%04ld", (long)time_val);
    pos += 4;

    // Latitude (9 digits): (lat × 1e5) + 1e8
    {
        int32_t lat_enc = clamp_i((int32_t)(t.latitude * 100000.0f + 100000000L), 0, 199999999L);
        sprintf(&out60[pos], "%09ld", (long)lat_enc);
        pos += 9;
    }

    // Longitude (9 digits): (lon × 1e5) + 1e8
    {
        int32_t lon_enc = clamp_i((int32_t)(t.longitude * 100000.0f + 100000000L), 0, 199999999L);
        sprintf(&out60[pos], "%09ld", (long)lon_enc);
        pos += 9;
    }

    // Speed GPS (3 digits): m/s × 10
    int32_t spd_enc = clamp_i((int32_t)(t.speed_gps * 10.0f), 0, 999);
    sprintf(&out60[pos], "%03ld", (long)spd_enc);
    pos += 3;

    // Heading GPS (3 digits): degrees 0–359
    int32_t hdg_enc = clamp_i((int32_t)t.heading_gps, 0, 999);
    sprintf(&out60[pos], "%03ld", (long)hdg_enc);
    pos += 3;

    // Altitude (5 digits): Kalman-filtered altitude in metres
    int32_t alt_enc = clamp_i((int32_t)t.altitude, 0, 99999);
    sprintf(&out60[pos], "%05ld", (long)alt_enc);
    pos += 5;

    // Voltage (3 digits): V × 100
    int32_t volt_enc = clamp_i((int32_t)(t.voltage * 100.0f), 0, 999);
    sprintf(&out60[pos], "%03ld", (long)volt_enc);
    pos += 3;

    // RSSI (3 digits): |RSSI| in dBm
    // RSSI is negative (e.g. -80), store as positive absolute value
    int32_t rssi_enc = clamp_i((int32_t)abs(rq.rssi), 0, 999);
    sprintf(&out60[pos], "%03ld", (long)rssi_enc);
    pos += 3;

    // Gain / SNR (3 digits): SNR × 10
    // SNR can be negative in poor conditions (-20 to +15 dB typical).
    // Offset by +200 so it always fits as a positive 3-digit number:
    //   -20 dB → (-200 + 200) = 000,  +15 dB → (150 + 200) = 350
    // Visualization decodes: data.Gain = stof(getNext(3)) / 10.0
    // To get true SNR on the decode side: SNR = (Gain * 10 - 200) / 10
    int32_t snr_enc = clamp_i((int32_t)(rq.snr * 10 + 200), 0, 999);
    sprintf(&out60[pos], "%03ld", (long)snr_enc);
    pos += 3;

    // Accel X (6 digits): (m/s² + 100) × 1000
    int32_t ax_enc = clamp_i((int32_t)((t.ax + 100.0f) * 1000.0f), 0, 199999);
    sprintf(&out60[pos], "%06ld", (long)ax_enc);
    pos += 6;

    // Accel Y (6 digits)
    int32_t ay_enc = clamp_i((int32_t)((t.ay + 100.0f) * 1000.0f), 0, 199999);
    sprintf(&out60[pos], "%06ld", (long)ay_enc);
    pos += 6;

    // Accel Z (6 digits)
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

// ── Receive one LoRa packet + radio quality ──────────────────────────────────
// Returns number of decoded payload bytes (0 on timeout/failure).
// rq is populated with RSSI and SNR from the module's AT response.
static size_t lora_receive_packet(uint8_t* out_data, size_t max_len,
                                   RadioQuality& rq,
                                   unsigned long rx_timeout_ms) {
    rq.rssi = 0;
    rq.snr  = 0;

    // Enter RX mode
    send_at("AT+TEST=RXLRPKT\r\n");
    wait_for("+TEST: RXLRPKT", 1000);

    // Wait for an incoming packet
    if (!wait_for("+TEST: RX", rx_timeout_ms)) {
        return 0;
    }

    // The "+TEST: RX" line is now in s_buf, but the RSSI/SNR line
    // is sent by the module shortly after.  Keep reading for 300 ms
    // to capture it.
    drain_extra(300);

    // ── Parse RSSI / SNR from the full response ──────────────────────────
    parse_rssi_snr(rq);

    // ── Extract hex payload from between the first pair of quotes ────────
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
    // 1. Receive LoRa packet + radio quality (blocks up to 10s)
    uint8_t      rx_buf[128];
    RadioQuality rq;
    size_t       rx_len = lora_receive_packet(rx_buf, sizeof(rx_buf), rq, 10000);

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
    Serial.print(F(" ms  GNSS: "));     Serial.print(telem.time_now);
    Serial.print(F("  Alt: "));         Serial.print(telem.altitude, 1);
    Serial.print(F(" m  VVel: "));      Serial.print(telem.vertical_velocity, 2);
    Serial.println(F(" m/s"));

    Serial.print(F("[GND] Temp: "));    Serial.print(telem.temperature, 1);
    Serial.print(F(" C  Pres: "));      Serial.print(telem.pressure, 1);
    Serial.print(F(" hPa  Volt: "));    Serial.print(telem.voltage, 2);
    Serial.println(F(" V"));

    Serial.print(F("[GND] GPS: "));
    Serial.print(telem.latitude, 5);    Serial.print(F(", "));
    Serial.print(telem.longitude, 5);   Serial.print(F("  Spd: "));
    Serial.print(telem.speed_gps, 1);   Serial.print(F(" m/s  Hdg: "));
    Serial.println(telem.heading_gps);

    Serial.print(F("[GND] Accel: "));
    Serial.print(telem.ax, 2); Serial.print(F(", "));
    Serial.print(telem.ay, 2); Serial.print(F(", "));
    Serial.print(telem.az, 2); Serial.println(F(" m/s2"));

    Serial.print(F("[GND] Radio: RSSI="));
    Serial.print(rq.rssi);
    Serial.print(F(" dBm  SNR="));
    Serial.print(rq.snr);
    Serial.println(F(" dB"));

    Serial.print(F("[GND] Flags: BMP="));
    Serial.print((telem.flags & 0x01) ? F("OK") : F("FAIL"));
    Serial.print(F("  IMU="));
    Serial.print((telem.flags & 0x02) ? F("OK") : F("FAIL"));
    Serial.print(F("  GNSS="));
    Serial.println((telem.flags & 0x04) ? F("OK") : F("FAIL"));

#else
    // ── FLIGHT MODE: 25-byte binary for simple-terminal-visualization ─────
    char decimal[OUTPUT_DIGITS + 1];
    encode_to_decimal(telem, rq, decimal);

    uint8_t output[OUTPUT_BYTES];
    decimal_to_bytes(decimal, output, OUTPUT_BYTES);

    Serial.write(output, OUTPUT_BYTES);
    Serial.flush();
#endif

    digitalWrite(LED_BUILTIN, HIGH);
}

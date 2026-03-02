// ── HAB Ground Station Receiver ───────────────────────────────────────────────
// Target board: Arduino UNO (ATmega328P)
//
// Hardware Serial (pins 0/1) → USB to PC
// SoftwareSerial             → LoRa module (AT commands, 9600 baud)
//
// Receives 23-byte LoRa telemetry packets, decodes them using
// telemetry_decoder, measures RSSI/SNR locally, and sends a single
// CSV line per packet over USB Serial.
//
// CSV format (human-readable AND machine-parseable):
//   $HAB,HHMMSS,lat,lon,spd,hdg,alt,volt,rssi,snr,ax,ay,az
//
// This eliminates the need for a DEBUG_MODE switch — the same output
// works in both Arduino Serial Monitor and the PC visualization app.
//
// Wiring:
//   LoRa TX  → UNO pin 2 (SOFT_RX_PIN)
//   LoRa RX  → UNO pin 7 (SOFT_TX_PIN)
//   LoRa VCC → 3.3V,  GND → GND
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "telemetry_decoder.h"

// ── Pin config ───────────────────────────────────────────────────────────────
#define SOFT_RX_PIN     2
#define SOFT_TX_PIN     7

// ── Radio config (must match flight computer) ────────────────────────────────
#define LORA_BAUD       9600
#define LORA_RF_CONFIG  "866,SF12,125,12,15,14,ON,OFF,OFF"

// ── USB Serial baud ──────────────────────────────────────────────────────────
#define SERIAL_BAUD     115200

// ── Packet sizes ─────────────────────────────────────────────────────────────
static const int TELEM_DIGITS      = 54;
static const int TELEM_PACKET_SIZE = 23;

// ── SoftwareSerial for LoRa ──────────────────────────────────────────────────
SoftwareSerial loraSerial(SOFT_RX_PIN, SOFT_TX_PIN);

// ── AT command buffer ────────────────────────────────────────────────────────
static char s_buf[384];
static int  s_buf_len = 0;

// ── AT command helpers ───────────────────────────────────────────────────────

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

    // Extract hex payload between quotes
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

    Serial.println(F("[GND] HAB Ground Station Receiver"));
    Serial.println(F("[GND] Initialising LoRa..."));

    if (!lora_init_ground()) {
        Serial.println(F("[GND] ERROR: LoRa init failed!"));
        while (true) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(200);
            digitalWrite(LED_BUILTIN, LOW);
            delay(200);
        }
    }

    Serial.println(F("[GND] LoRa ready. Listening..."));
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    // 1. Receive LoRa packet + radio quality
    uint8_t      rx_buf[128];
    RadioQuality rq;
    size_t       rx_len = lora_receive_packet(rx_buf, sizeof(rx_buf), rq, 10000);

    if (rx_len < (size_t)TELEM_PACKET_SIZE) {
        Serial.println(F("[GND] RX timeout or short packet"));
        return;
    }

    digitalWrite(LED_BUILTIN, LOW);     // blink on packet

    // 2. Decode 23 bytes → 54-digit decimal string
    char dec54[TELEM_DIGITS + 1];
    bytes_to_decimal(rx_buf, TELEM_PACKET_SIZE, dec54, TELEM_DIGITS);

    // 3. Parse into struct
    TelemetryData telem;
    parse_telemetry(dec54, telem);

    // 4. Send CSV line — works for both serial monitor and PC app
    char csv_line[256];
    format_csv_line(telem, rq, csv_line, sizeof(csv_line));
    Serial.println(csv_line);

    digitalWrite(LED_BUILTIN, HIGH);
}

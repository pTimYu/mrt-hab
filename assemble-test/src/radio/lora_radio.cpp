#include "lora_radio.h"
#include "../config.h"

// ── Internal serial and buffer ────────────────────────────────────────────────
// ESP32 / Arduino Nano ESP32 / Nano Matter have hardware UARTs.
// Serial1 can be mapped to any pair of GPIO pins at runtime.
#define LORA_SERIAL  Serial1

static char s_buf[384];   // AT response buffer
static int  s_buf_len = 0; // current length of valid data in s_buf

// ── Hex conversion helpers (binary bytes ↔ hex string) ────────────────────────

// Encode `len` raw bytes into a hex string.
//   e.g. {0x41, 0x0F, 0xFF} → "410FFF"
// `out` must be at least (len * 2 + 1) bytes.
static void bytes_to_hex(const uint8_t* in, size_t len, char* out) {
    for (size_t i = 0; i < len; i++) {
        sprintf(&out[i * 2], "%02X", in[i]);
    }
    out[len * 2] = '\0';
}

// Decode a hex string into raw bytes.
//   e.g. "410FFF" → {0x41, 0x0F, 0xFF}, returns 3
// Returns number of decoded bytes, or 0 on error.
static size_t hex_to_bytes(const char* hex, uint8_t* out, size_t max_len) {
    size_t hex_len = strlen(hex);
    if (hex_len % 2 != 0) return 0;           // must be even
    size_t n = hex_len / 2;
    if (n > max_len) return 0;                 // overflow guard

    for (size_t i = 0; i < n; i++) {
        char pair[3] = { hex[i * 2], hex[i * 2 + 1], '\0' };
        out[i] = (uint8_t)strtol(pair, NULL, 16);
    }
    return n;
}

// ── AT command helpers ────────────────────────────────────────────────────────

static void send_at(const char* cmd) {
    LORA_SERIAL.print(cmd);
#if TEST_MODE
    Serial.print(cmd);
#endif
}

// Block until `keyword` appears in the module response or `timeout` expires.
// The full response is captured in s_buf for later parsing.
static bool wait_for(const char* keyword, unsigned long timeout) {
    unsigned long start = millis();
    s_buf_len = 0;
    memset(s_buf, 0, sizeof(s_buf));

    while (millis() - start < timeout) {
        while (LORA_SERIAL.available()) {
            if (s_buf_len < (int)(sizeof(s_buf) - 1)) {
                s_buf[s_buf_len++] = LORA_SERIAL.read();
#if TEST_MODE
                Serial.print(s_buf[s_buf_len - 1]);
#endif
                delay(2);          // give UART time between bytes
            } else {
                LORA_SERIAL.read();     // discard overflow
            }
        }
        if (strstr(s_buf, keyword)) return true;
    }
    return false;
}

// Continue reading bytes into s_buf (without clearing) for up to `extra_ms`
// milliseconds.  Used to capture trailing RSSI/SNR data after the payload
// line has already been matched by wait_for().
static void drain_extra(unsigned long extra_ms) {
    unsigned long start = millis();
    while (millis() - start < extra_ms) {
        while (LORA_SERIAL.available()) {
            if (s_buf_len < (int)(sizeof(s_buf) - 1)) {
                s_buf[s_buf_len++] = LORA_SERIAL.read();
#if TEST_MODE
                Serial.print(s_buf[s_buf_len - 1]);
#endif
                delay(2);
            } else {
                LORA_SERIAL.read();
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
// Some modules use a comma-separated variant:
//   +TEST: RX "<hex>", RSSI:-80, SNR:10
//
// This function scans s_buf for "RSSI:" and "SNR:" and extracts the integers.
static void parse_rssi_snr(int& rssi, int& snr) {
    rssi = 0;
    snr  = 0;

    const char* p = strstr(s_buf, "RSSI:");
    if (p) {
        p += 5;                   // skip "RSSI:"
        rssi = atoi(p);           // handles leading minus sign
    }

    p = strstr(s_buf, "SNR:");
    if (p) {
        p += 4;                   // skip "SNR:"
        snr = atoi(p);
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

bool lora_init() {
    LORA_SERIAL.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

    // Basic AT check
    send_at("AT\r\n");
    if (!wait_for("+AT: OK", 500)) return false;

    // Enter P2P test mode
    send_at("AT+MODE=TEST\r\n");
    wait_for("+MODE: TEST", 500);

    // RF configuration — edit the string in config.h if you need different
    // frequency / SF / BW / etc.
    char cfg_cmd[128];
    snprintf(cfg_cmd, sizeof(cfg_cmd),
             "AT+TEST=RFCFG,%s\r\n", LORA_RF_CONFIG);
    send_at(cfg_cmd);
    wait_for("+TEST: RFCFG", 500);

    return true;
}

bool lora_send(const uint8_t* data, size_t len) {
    if (len == 0 || len > LORA_MAX_PAYLOAD) return false;

    // Hex-encode the raw bytes
    char hex_payload[LORA_MAX_PAYLOAD * 2 + 1];
    bytes_to_hex(data, len, hex_payload);

    // Build AT command:  AT+TEST=TXLRPKT,"<hex>"\r\n
    char cmd[LORA_MAX_PAYLOAD * 2 + 40];
    snprintf(cmd, sizeof(cmd), "AT+TEST=TXLRPKT,\"%s\"\r\n", hex_payload);

#if TEST_MODE
    Serial.print(F("[LoRa TX] len="));
    Serial.print(len);
    Serial.print(F("  hex="));
    Serial.println(hex_payload);
#endif

    send_at(cmd);

    if (wait_for("TX DONE", 2000)) {
#if TEST_MODE
        Serial.println(F("[LoRa] TX OK"));
#endif
        return true;
    }

#if TEST_MODE
    Serial.println(F("[LoRa] TX FAIL"));
#endif
    return false;
}

bool lora_receive(uint8_t* out_data, size_t max_len, size_t& out_len,
                  unsigned long rx_timeout_ms,
                  int* out_rssi,
                  int* out_snr) {
    out_len = 0;
    if (out_rssi) *out_rssi = 0;
    if (out_snr)  *out_snr  = 0;

    // Enter RX mode
    send_at("AT+TEST=RXLRPKT\r\n");
    wait_for("+TEST: RXLRPKT", 1000);

    // Wait for an incoming packet
    if (!wait_for("+TEST: RX", rx_timeout_ms)) {
#if TEST_MODE
        Serial.println(F("[LoRa] RX timeout"));
#endif
        return false;
    }

    // The "+TEST: RX" line is in s_buf, but the RSSI/SNR line may still
    // be arriving.  Read for another 200 ms to capture trailing data.
    drain_extra(200);

#if TEST_MODE
    Serial.println(F("[LoRa] RX packet:"));
    Serial.println(s_buf);
#endif

    // Response format:  +TEST: RX "<hex_payload>"
    // Extract the hex between the first pair of double-quotes.
    char* start = strchr(s_buf, '"');
    if (!start) return false;
    start++;                               // skip opening quote

    char* end = strchr(start, '"');
    if (!end) return false;
    *end = '\0';                           // null-terminate hex string

    // Decode hex → raw bytes
    out_len = hex_to_bytes(start, out_data, max_len);
    if (out_len == 0) {
#if TEST_MODE
        Serial.println(F("[LoRa] RX decode failed"));
#endif
        return false;
    }

    // ── Parse RSSI / SNR from the remainder of s_buf ──────────────────────
    // Restore the quote so the full buffer is searchable again, then parse.
    *end = '"';

    if (out_rssi || out_snr) {
        int rssi_val = 0, snr_val = 0;
        parse_rssi_snr(rssi_val, snr_val);
        if (out_rssi) *out_rssi = rssi_val;
        if (out_snr)  *out_snr  = snr_val;

#if TEST_MODE
        Serial.print(F("[LoRa] RSSI="));
        Serial.print(rssi_val);
        Serial.print(F(" dBm  SNR="));
        Serial.print(snr_val);
        Serial.println(F(" dB"));
#endif
    }

#if TEST_MODE
    Serial.print(F("[LoRa] RX decoded len="));
    Serial.println(out_len);
#endif
    return true;
}

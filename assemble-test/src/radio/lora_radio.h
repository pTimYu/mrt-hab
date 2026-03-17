#pragma once

#include <Arduino.h>

// ── LoRa Radio Module (AT command based, e.g. Wio-E5 / RYLR896) ─────────────
//
// Communicates with the LoRa module over SoftwareSerial using AT commands
// in TEST (P2P) mode.
//
// All send/receive functions operate on raw binary bytes (uint8_t).
// Internally, bytes are hex-encoded for the AT+TEST=TXLRPKT command
// and hex-decoded on receive.
//
// Usage:
//   lora_init();                              // in setup(), after Serial
//   lora_send(payload, len);                  // transmit raw bytes
//   lora_receive(buf, sizeof(buf), rxLen);    // receive raw bytes
//   int rssi; int snr;
//   lora_receive(buf, sizeof(buf), rxLen, 5000, &rssi, &snr);
// ─────────────────────────────────────────────────────────────────────────────

// Maximum payload size in bytes (the AT command hex string will be 2× this)
#define LORA_MAX_PAYLOAD  120

// Initialise SoftwareSerial and configure the LoRa module into TEST/P2P mode.
// Returns true if the module responds to AT commands and is configured.
bool lora_init();

// Transmit `len` raw bytes.  Each byte is hex-encoded and sent via
// AT+TEST=TXLRPKT.  Blocks until "TX DONE" or timeout.
// Returns true on success.
bool lora_send(const uint8_t* data, size_t len);

// Enter RX mode and wait for one packet (up to `rx_timeout_ms`).
// On success, decoded raw bytes are written to `out_data` and `out_len`
// is set to the number of bytes received.
//
// Optional: pass non-null pointers for out_rssi / out_snr to retrieve
// the RSSI (dBm, typically negative) and SNR (dB) reported by the
// LoRa module for the received packet.  If the module response does
// not contain RSSI/SNR fields, the values are set to 0.
//
// Returns true if a valid packet was received within the timeout.
bool lora_receive(uint8_t* out_data, size_t max_len, size_t& out_len,
                  unsigned long rx_timeout_ms = 5000,
                  int* out_rssi = nullptr,
                  int* out_snr  = nullptr);

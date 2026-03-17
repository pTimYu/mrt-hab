#include "buzzer.h"
#include "../config.h"
#include <Arduino.h>

// ── Morse code table — digits 0-9 ─────────────────────────────────────────────
static const char* const MORSE_DIGITS[10] = {
    "-----",   // 0
    ".----",   // 1
    "..---",   // 2
    "...--",   // 3
    "....-",   // 4
    ".....",   // 5
    "-....",   // 6
    "--...",   // 7
    "---..",   // 8
    "----."    // 9
};

// ── Morse code table — letters (A=0, B=1 ... Z=25) ───────────────────────────
static const char* const MORSE_LETTERS[26] = {
    ".-",      // A
    "-...",    // B
    "-.-.",    // C
    "-..",     // D
    ".",       // E
    "..-.",    // F
    "--.",     // G
    "....",    // H
    "..",      // I
    ".---",    // J
    "-.-",     // K
    ".-..",    // L
    "--",      // M
    "-.",      // N
    "---",     // O
    ".--.",    // P
    "--.-",    // Q
    ".-.",     // R
    "...",     // S
    "-",       // T
    "..-",     // U
    "...-",    // V
    ".--",     // W
    "-..-",    // X
    "-.--",    // Y
    "--.."     // Z
};

// ── Private helpers ───────────────────────────────────────────────────────────
static void play_sequence(const char* morse) {
    for (const char* p = morse; *p; p++) {
        (*p == '.') ? buzzer_dot() : buzzer_dash();
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

void buzzer_init() {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // LEDs — active-LOW on Arduino Nano Matter (LOW = on, HIGH = off)
    pinMode(LEDR, OUTPUT); digitalWrite(LEDR, HIGH); // off
    pinMode(LEDG, OUTPUT); digitalWrite(LEDG, HIGH); // off
    pinMode(LEDB, OUTPUT); digitalWrite(LEDB, HIGH); // off
}

void buzzer_dot() {
#if ENABLE_BUZZER
    digitalWrite(BUZZER_PIN, HIGH);
#endif
    digitalWrite(LEDR, LOW);    // on
    delay(100);
#if ENABLE_BUZZER
    digitalWrite(BUZZER_PIN, LOW);
#endif
    digitalWrite(LEDR, HIGH);   // off
    delay(100);
}

void buzzer_dash() {
#if ENABLE_BUZZER
    digitalWrite(BUZZER_PIN, HIGH);
#endif
    digitalWrite(LEDR, LOW);    // on
    delay(500);
#if ENABLE_BUZZER
    digitalWrite(BUZZER_PIN, LOW);
#endif
    digitalWrite(LEDR, HIGH);   // off
    delay(100);
}

void buzzer_digit(int digit) {
    if (digit < 0 || digit > 9) return;
    play_sequence(MORSE_DIGITS[digit]);
}

void buzzer_error(int device_code) {
    if (device_code < 0 || device_code > 9) return;
    // Play twice so it's easy to catch
    play_sequence(MORSE_DIGITS[device_code]);
    delay(500);
    play_sequence(MORSE_DIGITS[device_code]);
}

void buzzer_ok() {
    // Blue LED on = all systems go
    digitalWrite(LEDB, LOW);   // on

    // O  (--- in Morse)
    play_sequence(MORSE_LETTERS['O' - 'A']);
    delay(300);

    // K  (-.- in Morse)
    play_sequence(MORSE_LETTERS['K' - 'A']);
}

void buzzer_land() {
    // L  (.-..)
    play_sequence(MORSE_LETTERS['L' - 'A']);
    delay(300);

    // A  (.-)
    play_sequence(MORSE_LETTERS['A' - 'A']);
    delay(300);

    // N  (-.)
    play_sequence(MORSE_LETTERS['N' - 'A']);
    delay(300);

    // D  (-..)
    play_sequence(MORSE_LETTERS['D' - 'A']);
}

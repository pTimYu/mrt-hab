#include "flight_controller.h"
#include "../config.h"
#include "../io/buzzer.h"
#include "../utils/data_saver.h"
#include <Arduino.h>

// ── Internal state ────────────────────────────────────────────────────────────
static FlightStatus s_status          = STNDBY;
static uint32_t     s_prep_start_ms   = 0;   // millis() at boot
static uint32_t     s_takeoff_ms      = 0;   // millis() at takeoff
static float        s_ground_altitude = 0.0f;
static int          s_stationary_count = 0;
static bool         s_flight_terminated = false;
static bool         s_file_closed     = false;

// ── Private helpers ───────────────────────────────────────────────────────────
static void fire_actuators() {
    digitalWrite(ACTUATOR_PIN_1, HIGH);
    digitalWrite(ACTUATOR_PIN_2, HIGH);
    // Actuators stay latched HIGH — they are cut-down pyros or similar.
    // Do not drive LOW here; the flight_controller owns this latch.
}

// ── Public API ────────────────────────────────────────────────────────────────
void fc_init() {
    s_status            = STNDBY;
    s_prep_start_ms     = millis();
    s_takeoff_ms        = 0;
    s_ground_altitude   = 0.0f;
    s_stationary_count  = 0;
    s_flight_terminated = false;
    s_file_closed       = false;

    pinMode(ACTUATOR_PIN_1, OUTPUT);
    pinMode(ACTUATOR_PIN_2, OUTPUT);
    digitalWrite(ACTUATOR_PIN_1, LOW);
    digitalWrite(ACTUATOR_PIN_2, LOW);

    // In TEST_MODE, briefly fire actuators at boot to confirm they work
#if TEST_MODE
    digitalWrite(ACTUATOR_PIN_1, HIGH);
    digitalWrite(ACTUATOR_PIN_2, HIGH);
    delay(5000);
    digitalWrite(ACTUATOR_PIN_1, LOW);
    digitalWrite(ACTUATOR_PIN_2, LOW);
#endif
}

void fc_update(DataSet& data) {

    uint32_t now = millis();

    // ── Write effective flight time into DataSet ───────────────────────────
    if (s_status == STNDBY) {
        data.time = 0;
    } else {
        data.time = now - s_takeoff_ms;
    }

    switch (s_status) {

        // ── STANDBY ───────────────────────────────────────────────────────
        case STNDBY: {
            // Keep ground altitude updated until we lift off
            s_ground_altitude = data.altitude;

            // Wait for sensor readings to settle before detecting takeoff
            uint32_t prep_elapsed = now - s_prep_start_ms;
            if (prep_elapsed > STANDBY_SETTLE_MS) {
                if (data.vertical_velocity > TAKEOFF_THRESHOLD) {
                    s_status    = ASCENT;
                    s_takeoff_ms = now;

#if TEST_MODE
                    Serial.println(F(">> TAKEOFF DETECTED"));
#endif
                    buzzer_dash(); // single dash = state change
                }
            }
            break;
        }

        // ── ASCENT ────────────────────────────────────────────────────────
        case ASCENT: {
            // Log every loop during flight
            data_saver_write(data);

            // Flight termination: time limit OR altitude limit
            float alt_above_ground = data.altitude - s_ground_altitude;
            bool time_exceeded     = (data.time > MAX_FLIGHT_TIME);
            bool alt_exceeded      = (alt_above_ground > MAX_FLIGHT_HEIGHT);

            if ((time_exceeded || alt_exceeded) && !s_flight_terminated) {
                fire_actuators();
                s_flight_terminated = true;

#if TEST_MODE
                Serial.println(F(">> FLIGHT TERMINATION FIRED"));
#endif
                buzzer_dash();
            }

            // Transition to descent when climbing stops
            if (data.vertical_velocity < 0.01f) {
                s_status = DESCNT;

#if TEST_MODE
                Serial.println(F(">> DESCENT DETECTED"));
#endif
                buzzer_dash();
            }
            break;
        }

        // ── DESCENT ───────────────────────────────────────────────────────
        case DESCNT: {
            // Log every loop during descent
            data_saver_write(data);

            // Landing detection: sufficiently still for N consecutive loops
            if (fabsf(data.vertical_velocity) < 0.1f) {
                s_stationary_count++;
            } else {
                s_stationary_count = 0;
            }

            if (s_stationary_count > LANDING_THRESHOLD) {
                s_status = LANDED;

#if TEST_MODE
                Serial.println(F(">> LANDING DETECTED"));
#endif
            }
            break;
        }

        // ── LANDED ────────────────────────────────────────────────────────
        case LANDED: {
            // Close SD file once on first entry
            if (!s_file_closed) {
                data_saver_flush();
                data_saver_close();
                s_file_closed = true;
            }

            // Broadcast "LAND" in Morse every 60 s indefinitely
            buzzer_land();
            delay(60000);
            break;
        }

        default:
            break;
    }

#if TEST_MODE
    // ── Verbose serial output (all states) ───────────────────────────────
    Serial.print(F("STATE: "));
    switch (s_status) {
        case STNDBY: Serial.print(F("STNDBY")); break;
        case ASCENT: Serial.print(F("ASCENT")); break;
        case DESCNT: Serial.print(F("DESCNT")); break;
        case LANDED: Serial.print(F("LANDED")); break;
    }
    Serial.print(F("  ALT: "));  Serial.print(data.altitude, 2);
    Serial.print(F("  VVEL: ")); Serial.print(data.vertical_velocity, 3);
    Serial.print(F("  GND: "));  Serial.print(s_ground_altitude, 2);
    Serial.print(F("  T: "));    Serial.print(data.time);
    Serial.print(F("  BMP: "));  Serial.print(data.bmp  ? F("OK") : F("FAIL"));
    Serial.print(F("  IMU: "));  Serial.println(data.imu ? F("OK") : F("FAIL"));
#endif
}

FlightStatus fc_get_status() {
    return s_status;
}

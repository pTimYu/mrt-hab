#include "kalman_filter.h"

// ── Tuning constants ──────────────────────────────────────────────────────────
//
// MEASUREMENT_NOISE  (R)
//   How much we distrust the barometer reading.
//   Larger value → filter trusts the IMU prediction more → smoother but slower
//   to react to real altitude changes. 2.0 m² matches the legacy value.
//
// PROCESS_NOISE  (Q)
//   How much we expect the real state to drift between steps.
//   Larger value → filter reacts faster to barometer corrections but is noisier.
//   0.05 matches the legacy value.
// ─────────────────────────────────────────────────────────────────────────────
static const float MEASUREMENT_NOISE = 2.0f;   // R  (m²)
static const float PROCESS_NOISE     = 0.05f;  // Q  (m²/s² per step)

// ── Internal state ───────────────────────────────────────────────────────────
static float s_altitude = 0.0f;
static float s_velocity = 0.0f;

// 2×2 error covariance matrix  P = [[p00, p01], [p10, p11]]
static float s_p00 = 1.0f, s_p01 = 0.0f;
static float s_p10 = 0.0f, s_p11 = 1.0f;

// ── Public API ───────────────────────────────────────────────────────────────

void kalman_init() {
    s_altitude = 0.0f;
    s_velocity = 0.0f;
    s_p00 = 1.0f;  s_p01 = 0.0f;
    s_p10 = 0.0f;  s_p11 = 1.0f;
}

void kalman_update(float baro_altitude, bool baro_valid,
                   float accel_z,       bool imu_valid,
                   float dt) {

    // If IMU failed, assume zero acceleration (constant-velocity model).
    // The filter degrades gracefully — velocity estimate drifts but
    // barometer corrections will keep altitude reasonable.
    if (!imu_valid) accel_z = 0.0f;

    // ── 1. Predict ────────────────────────────────────────────────────────
    float alt_pred = s_altitude + s_velocity * dt + 0.5f * accel_z * dt * dt;
    float vel_pred = s_velocity + accel_z * dt;

    // Covariance extrapolation  P_pred = F*P*F' + Q
    float p00_pred = s_p00 + (s_p10 + s_p01) * dt + s_p11 * dt * dt + PROCESS_NOISE;
    float p01_pred = s_p01 + s_p11 * dt;
    float p10_pred = s_p10 + s_p11 * dt;
    float p11_pred = s_p11 + PROCESS_NOISE;

    if (!baro_valid) {
        // ── Predict-only mode ─────────────────────────────────────────────
        // No barometer reading available this cycle — skip the measurement
        // update and commit the predicted state directly.
        // The covariance grows, meaning the next valid barometer reading
        // will be trusted more heavily to correct the drift.
        s_altitude = alt_pred;
        s_velocity = vel_pred;
        s_p00 = p00_pred;
        s_p01 = p01_pred;
        s_p10 = p10_pred;
        s_p11 = p11_pred;
        return;
    }

    // ── 2. Kalman Gain ────────────────────────────────────────────────────
    // H = [1, 0]  — we observe altitude only
    float S  = p00_pred + MEASUREMENT_NOISE;
    float k0 = p00_pred / S;
    float k1 = p10_pred / S;

    // ── 3. Update ─────────────────────────────────────────────────────────
    float residual = baro_altitude - alt_pred;

    s_altitude = alt_pred + k0 * residual;
    s_velocity = vel_pred + k1 * residual;

    // ── 4. Update Error Covariance  P = (I - K*H) * P_pred ───────────────
    s_p00 = (1.0f - k0) * p00_pred;
    s_p01 = (1.0f - k0) * p01_pred;
    s_p10 = p10_pred - k1 * p00_pred;
    s_p11 = p11_pred - k1 * p01_pred;
}

float kalman_get_altitude() { return s_altitude; }
float kalman_get_velocity() { return s_velocity; }

#include "orientation_filter.h"
#include <cmath>


// ---------- your existing Kalman class (unchanged) ----------
class Kalman {
public:
    Kalman() {
        angle = 0.0f; 
        bias = 0.0f;
        P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0.0f;
    }

    void setR(float newR) { R_measure = newR; }
    
    float getAngle(float newAngle, float newRate, float dt) {
        // 1. Predict state
        rate = newRate - bias;
        angle += dt * rate;

        // 2. Update estimation error covariance
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_gyroBias * dt;

        // 3. Calculate Kalman gain
        float S = P[0][0] + R_measure;
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        // 4. Calculate angle and bias (The "Revised" data)
        float y = newAngle - angle;
        angle += K[0] * y;
        bias += K[1] * y;

        // 5. Update error covariance
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    };

    float getRate() { return rate; }

private:
    float Q_angle = 0.003f;
    float Q_gyroBias = 0.01f;
    float R_measure = 0.008f;

    float angle; 
    float bias; 
    float rate;
    float P[2][2]; 
};

// Global filter instances (for one sensor)
Kalman kalmanX; 
Kalman kalmanY;

// -------- helpers for gravity removal ----------
static constexpr float G_MSS = 10.0f; // meters per second^2
inline float deg2rad(float d) { return d * 0.017453292519943295f; }

// Compute gravity in body frame from roll (deg) and pitch (deg)
static void gravityFromRollPitch(float roll_deg, float pitch_deg, float &gx, float &gy, float &gz) {
    float phi   = deg2rad(roll_deg);   // roll
    float theta = deg2rad(pitch_deg);  // pitch

    float sinPhi = sinf(phi);
    float cosPhi = cosf(phi);
    float sinTheta = sinf(theta);
    float cosTheta = cosf(theta);

    // Gravity components in body frame (assuming inertial gravity [0,0,g] in nav frame)
    gx = -G_MSS * sinTheta;
    gy =  G_MSS * sinPhi * cosTheta;
    gz =  G_MSS * cosPhi * cosTheta;
}

// -------- modified processIMU: outputs linear acceleration (gravity removed) --------
OrientationData processIMU(float accX, float accY, float accZ, float gyroX, float gyroY, float dt) {
    float accMag = sqrt(accX*accX + accY*accY + accZ*accZ);

    if (fabs(accMag - 9.80665f) < 0.3f) {
        kalmanX.setR(0.008f);
        kalmanY.setR(0.008f);
    } else {
        kalmanX.setR(0.3f);   // ignore accel during strong motion
        kalmanY.setR(0.3f);
    }
    
    // 1. Calculate Roll and Pitch from Accelerometer (using trigonometry)
    float accRoll  = atan2f(accY, accZ) * 57.29578f; // deg
    float accPitch = atanf(-accX / sqrtf(accY * accY + accZ * accZ)) * 57.29578f; // deg

    // 2. Filter the data
    OrientationData out;
    out.roll  = kalmanX.getAngle(accRoll, gyroX, dt);
    out.pitch = kalmanY.getAngle(accPitch, gyroY, dt);
    
    // 3. Revised gyro data (unbiased)
    out.gx = kalmanX.getRate();
    out.gy = kalmanY.getRate();

    // 4. Compute gravity vector from the filtered orientation and remove it from accel
    float gx, gy, gz;
    gravityFromRollPitch(out.roll, out.pitch, gx, gy, gz);

    // NOTE on units:
    // - If accX/Y/Z are in m/s^2 (preferred), subtract gx/gy/gz directly.
    // - If accX/Y/Z are in "g" units (1.0 == 1g), multiply by G_MSS first:
    //     float ax_mss = accX * G_MSS;
    //   then do ax_mss - gx (and similarly for y,z).
    //
    // Here we assume accX/Y/Z are in m/s^2. If they are in g, uncomment the conversions below:
    // accX *= G_MSS; accY *= G_MSS; accZ *= G_MSS;

    out.ax = accX - gx;
    out.ay = accY - gy;
    out.az = accZ - gz;

    return out;
}
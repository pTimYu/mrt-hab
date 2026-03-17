#pragma once

struct OrientationData {
    float roll, pitch;
    float gx, gy;
    float ax, ay, az;
};

OrientationData processIMU(float accX, float accY, float accZ,
                            float gyroX, float gyroY, float dt);
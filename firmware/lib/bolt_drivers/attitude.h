// Mahony attitude filter (quaternion) + gyro bias estimation.  Body frame x fwd, y left, z up.
#pragma once
#include <math.h>

namespace att {

class Mahony {
 public:
  float kp = 1.5f, ki = 0.02f;
  float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
  float bx = 0, by = 0, bz = 0;          // integral feedback (gyro bias)

  // gyro rad/s, accel any unit (normalised), dt s
  void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    const float an = sqrtf(ax * ax + ay * ay + az * az);
    // only trust the accelerometer when it reads ~1 g (not during jumps / impacts)
    if (an > 7.0f && an < 12.5f) {
      ax /= an; ay /= an; az /= an;
      const float vx = 2 * (q1 * q3 - q0 * q2);
      const float vy = 2 * (q0 * q1 + q2 * q3);
      const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
      const float ex = ay * vz - az * vy, ey = az * vx - ax * vz, ez = ax * vy - ay * vx;
      bx += ki * ex * dt; by += ki * ey * dt; bz += ki * ez * dt;
      gx += kp * ex + bx; gy += kp * ey + by; gz += kp * ez + bz;
    } else {
      gx += bx; gy += by; gz += bz;
    }
    const float h = 0.5f * dt;
    const float a = q0, b = q1, c = q2;
    q0 += (-b * gx - c * gy - q3 * gz) * h;
    q1 += (a * gx + c * gz - q3 * gy) * h;
    q2 += (a * gy - b * gz + q3 * gx) * h;
    q3 += (a * gz + b * gy - c * gx) * h;
    const float n = 1.f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= n; q1 *= n; q2 *= n; q3 *= n;
  }

  // roll about +x, pitch about +y (nose DOWN positive), yaw about +z
  void euler(float& roll, float& pitch, float& yaw) const {
    roll = atan2f(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
    float s = 2 * (q0 * q2 - q3 * q1);
    s = s > 1 ? 1 : (s < -1 ? -1 : s);
    pitch = asinf(s);
    yaw = atan2f(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
  }

  // initialise roll/pitch straight from the accelerometer (robot standing still)
  void init_from_accel(float ax, float ay, float az) {
    const float roll = atan2f(ay, az);
    const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    const float cr = cosf(roll / 2), sr = sinf(roll / 2), cp = cosf(pitch / 2), sp = sinf(pitch / 2);
    q0 = cr * cp; q1 = sr * cp; q2 = cr * sp; q3 = -sr * sp;
  }
};

}  // namespace att

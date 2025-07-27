#pragma once
#include <Arduino.h>

namespace common_utils {
  struct rpy_t {
    float roll;
    float pitch;
    float yaw;
  };

  struct quat_t {
    float w;
    float x;
    float y;
    float z;
  };

  inline rpy_t to_radians(rpy_t rpy) {
    rpy_t rad_rpy;
    rad_rpy.roll  = rpy.roll * DEG_TO_RAD;
    rad_rpy.pitch = rpy.pitch * DEG_TO_RAD;
    rad_rpy.yaw   = rpy.yaw * DEG_TO_RAD;
    return rad_rpy;
  }

  inline rpy_t to_degrees(rpy_t rpy) {
    rpy_t deg_rpy;
    deg_rpy.roll  = rpy.roll * RAD_TO_DEG;
    deg_rpy.pitch = rpy.pitch * RAD_TO_DEG;
    deg_rpy.yaw   = rpy.yaw * RAD_TO_DEG;
    return deg_rpy;
  }

  inline rpy_t to_rpy(quat_t q) {
    rpy_t rpy;
    rpy.roll  = atan2(2.0f * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
    rpy.pitch = asin(-2.0f * (q.x * q.z - q.w * q.y));
    rpy.yaw   = atan2(2.0f * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
    return rpy;
  }

  inline quat_t to_quat(rpy_t rpy) {
    quat_t q;
    float cy = cos(rpy.yaw * 0.5f);
    float sy = sin(rpy.yaw * 0.5f);
    float cp = cos(rpy.pitch * 0.5f);
    float sp = sin(rpy.pitch * 0.5f);
    float cr = cos(rpy.roll * 0.5f);
    float sr = sin(rpy.roll * 0.5f);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
  }

  inline rpy_t acc_rpy(float accX, float accY, float accZ) {
    rpy_t rpy;
    float Aroll  = atan2f(accY, accZ);
    float Apitch = -atan2f(accX, sqrtf(accY * accY + accZ * accZ));
    rpy.roll     = Aroll;
    rpy.pitch    = Apitch;
    rpy.yaw      = 0.0F;
    return rpy;
  }

  inline rpy_t gyro_rpy(rpy_t pre, float gyroX, float gyroY, float gyroZ, float dt) {
    pre.roll += gyroX * dt;
    pre.pitch += gyroY * dt;
    pre.yaw += gyroZ * dt;
    return pre;
  }
} // namespace common_utils

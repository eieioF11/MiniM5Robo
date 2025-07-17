#pragma once
#include <Arduino.h>

namespace common_lib
{
  struct rpy_t
  {
    float roll;
    float pitch;
    float yaw;
  };

  rpy_t acc_rpy(float accX, float accY, float accZ)
  {
    rpy_t rpy;
    float Aroll = atan2f(accY, accZ);
    float Apitch = -atan2f(accX, sqrtf(accY * accY + accZ * accZ));
    rpy.roll = Aroll;
    rpy.pitch = Apitch;
    rpy.yaw = 0.0F;
    return rpy;
  }

  rpy_t gyro_rpy(rpy_t pre, float gyroX, float gyroY, float gyroZ, float dt)
  {
    pre.roll += gyroX * dt;
    pre.pitch += gyroY * dt;
    pre.yaw += gyroZ * dt;
    return pre;
  }
} // namespace common_lib

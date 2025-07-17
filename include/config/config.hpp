#pragma once
#include <Arduino.h>
#include <M5Unified.h>
#include <tuple>
// math
#include "utility/math_util.hpp"
// filter
#include "filter/lowpass_filter.hpp"
#include "filter/complementary_filter.hpp"

constexpr float LPF_ALPHA = 0.88;
constexpr float COMP_ALPHA = 0.95;

// gyro
constexpr float CALIB_TIME = 2.0;
bool claib_flag = false;
int calib_count = 0;
uint32_t timer;
std::array<float, 3> gyro_offset = {-0.0014, 0.0045, 0.0};
void gyro_caliblation()
{
  float calib_time = (float)(micros() - timer) / 1000000;
  if (calib_time > CALIB_TIME)
    claib_flag = false;
  M5.Display.startWrite();
  M5.Display.setCursor(0, 0);
  M5.Display.printf("Gyro Calibration\n");
  M5.Display.printf("time:%4.3f\n", calib_time);
  float gx, gy, gz;
  M5.Imu.getGyro(&gx, &gy, &gz);
  M5.Display.printf("gyro(%5.1f,%5.1f,%5.1f)\n", gx, gy, gz);
  gyro_offset[0] += gx;
  gyro_offset[1] += gy;
  gyro_offset[2] += gz;
  calib_count++;
  gyro_offset[0] /= calib_count;
  gyro_offset[1] /= calib_count;
  gyro_offset[2] /= calib_count;
  // M5.Display.printf("offset(%5.4f,%5.4f,%5.4f)\n", gyro_offset[0], gyro_offset[1], gyro_offset[2]);
  M5.Display.printf("count:%d\n", calib_count);
  M5.Display.endWrite();
}

// filter
common_lib::LowpassFilterf lpf_acc_x(LPF_ALPHA);
common_lib::LowpassFilterf lpf_acc_y(LPF_ALPHA);
common_lib::ComplementaryFilterf comp_filter_x(COMP_ALPHA);
common_lib::ComplementaryFilterf comp_filter_y(COMP_ALPHA);

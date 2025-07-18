#pragma once
#include <Arduino.h>
#include <M5Unified.h>
#include "utility/dynamixel_utils.hpp"
#include <tuple>
#include <vector>
#include <memory>
#include <array>
#include <string>

// math
#include "utility/math_util.hpp"
// filter
#include "filter/lowpass_filter.hpp"
#include "filter/complementary_filter.hpp"
//AW9523
#include <Adafruit_AW9523.h>
#include "move/move_base.hpp"
#include "move/two_wheels.hpp"


#define DEBUG_SERIAL Serial
HardwareSerial &DXL_SERIAL = Serial2;
HardwareSerial &LIDAR_SERIAL = Serial1;

#define MICROROS_AGENT_PORT 8888
#define MICROROS_AGENT_IP "192.168.0.117" //※ HOST PC IP


#define SD_SPI_CS_PIN 4
#define SD_SWITCH_PIN 4
Adafruit_AW9523 aw;
void aw9523_begin()
{
  Wire.begin(12,11);
  if (! aw.begin(0x58, &Wire)) {
    Serial.println("AW9523 not found? Check wiring!");
    while (1) delay(10);  // halt forever
  }
  Serial.println("AW9523 found!");
  aw.pinMode(SD_SWITCH_PIN, INPUT);
}

bool sd_exist()
{
  return !aw.digitalRead(SD_SWITCH_PIN);
}

constexpr uint8_t LIDAR_RX = 9;
constexpr uint8_t LIDAR_TX = 8;

constexpr uint8_t RX_SERVO = 18; //9
constexpr uint8_t TX_SERVO = 17; //8

// 右 id
constexpr uint8_t DXL_ID_LW = 0;
// 左 id
constexpr uint8_t DXL_ID_RW = 1;

constexpr float DXL_PROTOCOL_VERSION = 2.0;

constexpr float MAX_RPM = 101.0;// M288
// constexpr float MAX_RPM = 370.0; // M077

constexpr float WHEEL_RADIUS = 0.0285; // [m]
constexpr float WHEEL_D = 48.0 * common_utils::constants::mm_to_m; // [m] 車輪間距離

#define GYRO_MIN_VALUE -0.005f
// モーターの設定
// mode
//  OP_CURRENT
//  OP_VELOCITY
//  OP_POSITION
//  OP_EXTENDED_POSITION
//  OP_CURRENT_BASED_POSITION
//  OP_PWM
// DXLMotor m_lw(DXL_ID_LW, OP_CURRENT);
DXLMotor m_lw(DXL_ID_LW, OP_VELOCITY);
// DXLMotor m_rw(DXL_ID_RW, OP_CURRENT);
DXLMotor m_rw(DXL_ID_RW, OP_VELOCITY);

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

std::shared_ptr<kinematics::MoveBasef> move_;

enum class DisplayMode
{
  NONE,
  AVATAR,
  RESET,
  INFO,
  OTA,
  LIDAR,
};
DisplayMode display_mode = DisplayMode::NONE;

#include <Avatar.h>
using namespace m5avatar;
Avatar avatar;

bool reset_flag = false;

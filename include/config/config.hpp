#pragma once
#include "utility/dynamixel_utils.hpp"
#include <Arduino.h>
#include <M5Unified.h>
#include <SD.h>
#include <array>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>
// math
#include "utility/math_util.hpp"
#include "utility/string_util.hpp"
// filter
#include "filter/complementary_filter.hpp"
#include "filter/lowpass_filter.hpp"
// AW9523
#include "move/move_base.hpp"
#include "move/two_wheels.hpp"
#include <Adafruit_AW9523.h>

#define DEBUG_SERIAL Serial
HardwareSerial& DXL_SERIAL   = Serial2;
HardwareSerial& LIDAR_SERIAL = Serial1;

#define MICROROS_AGENT_PORT 8888

#define SD_SPI_CS_PIN 4
#define SD_SWITCH_PIN 4
#define CONFIG_FILE "/config.csv"
Adafruit_AW9523 aw;
void aw9523_begin() {
  // Wire.begin(12, 11);
  if (!aw.begin(0x58, &Wire)) {
    Serial.println("AW9523 not found? Check wiring!");
    while (1)
      delay(10); // halt forever
  }
  Serial.println("AW9523 found!");
  aw.pinMode(SD_SWITCH_PIN, INPUT);
}

bool sd_exist() { return !aw.digitalRead(SD_SWITCH_PIN); }

constexpr uint8_t LIDAR_RX = 9;
constexpr uint8_t LIDAR_TX = 8;

constexpr uint8_t RX_SERVO = 18; // 9
constexpr uint8_t TX_SERVO = 17; // 8

// 右 id
constexpr uint8_t DXL_ID_LW = 0;
// 左 id
constexpr uint8_t DXL_ID_RW = 1;

constexpr float DXL_PROTOCOL_VERSION = 2.0;

constexpr float MAX_RPM = 101.0; // M288
// constexpr float MAX_RPM = 370.0; // M077

constexpr float WHEEL_RADIUS = 0.0285;                                  // [m]
constexpr float WHEEL_D      = 48.0 * common_utils::constants::mm_to_m; // [m] 車輪間距離

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
bool claib_flag            = false;
int calib_count            = 0;
uint32_t timer;
std::array<float, 3> gyro_offset = {-0.0014, 0.0045, 0.0};
void gyro_caliblation() {
  float calib_time = (float)(micros() - timer) / 1000000;
  if (calib_time > CALIB_TIME) claib_flag = false;
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

enum class DisplayMode {
  NONE = 0,
  AVATAR,
  LIDAR,
  CAMERA,
  INFO,
  RESET,
  OTA,
};
DisplayMode display_mode = DisplayMode::NONE;
void sift_display_mode(bool reverse = false) {
  static int mode = static_cast<int>(display_mode);
  if (reverse)
    mode--;
  else
    mode++;
  if (mode < static_cast<int>(DisplayMode::AVATAR)) mode = static_cast<int>(DisplayMode::INFO);
  if (mode > static_cast<int>(DisplayMode::INFO)) mode = static_cast<int>(DisplayMode::AVATAR);
  display_mode = static_cast<DisplayMode>(mode);
}

std::tuple<bool, std::string, std::string, std::string, int> get_config() {
  using namespace common_utils;
  bool sdexist = false;
  sdexist      = sd_exist();
  Serial.printf("SD CARD:%d\n", sdexist);
  if (sdexist) {
    while (!SD.begin(SD_SPI_CS_PIN, SPI, 25000000)) {
      Serial.println("SD CARD ERROR");
      M5.Display.printf("SD CARD ERROR\n");
      delay(1000);
    }
    if (SD.cardSize() == CARD_NONE) {
      Serial.println("SD CARD NOT PRESENT");
      M5.Display.printf("SD CARD NOT PRESENT\n");
      return std::make_tuple(false, "", "", "", 0);
    }
    if (!SD.exists(CONFIG_FILE)) {
      Serial.println("SD CARD CONFIG FILE NOT EXIST");
      M5.Display.printf("SD CARD CONFIG FILE NOT EXIST\n");
      return std::make_tuple(false, "", "", "", 0); // Return false if file does not exist
    }
    // ota設定
    File fp = SD.open(CONFIG_FILE);
    if (fp) {
      unsigned int cnt = 0;
      char data[64];
      char* str;
      bool flag = false;
      Serial.println("file reading");
      M5.Display.printf("file reading\n");
      while (fp.available()) {
        data[cnt++] = fp.read();
        flag        = true;
      }
      if (flag) {
        std::string str_data           = std::string(data);
        std::vector<std::string> str_l = split(str_data, "\n");
        if (str_l.size() >= 4) {
          std::vector<std::string> vec1 = split(str_l[0], ",");
          std::vector<std::string> vec2 = split(str_l[1], ",");
          std::vector<std::string> vec3 = split(str_l[2], ",");
          std::vector<std::string> vec4 = split(str_l[3], ",");
          if (vec1.size() >= 2 && vec2.size() >= 2 && vec3.size() >= 2 && vec4.size() >= 2) {
            if (vec1[0] == "SSID" && vec2[0] == "PASS" && vec3[0] == "AGENT_IP" && vec4[0] == "AGENT_PORT") {
              std::string ssid     = vec1[1];
              std::string password = vec2[1];
              std::string agent_ip = vec3[1];
              int agent_port       = std::stoi(vec4[1]);
              Serial.println("wifi info");
              Serial.printf("ssid:%s\n", ssid.c_str());
              Serial.printf("pass:%s\n", password.c_str());
              Serial.printf("agent_ip:%s\n", agent_ip.c_str());
              Serial.printf("agent_port:%d\n", agent_port);
              M5.Display.printf("read config success\n");
              return std::make_tuple(true, ssid, password, agent_ip, agent_port); // Return all as a tuple
            }
          }
        }
      } else
        Serial.println("file read error");
      fp.close();
    } else
      Serial.println("file open error");
  } else
    Serial.println("SD not exist");
  return std::make_tuple(false, "", "", "", 0);
}

#include <Avatar.h>
using namespace m5avatar;
Avatar avatar;

bool reset_flag = false;

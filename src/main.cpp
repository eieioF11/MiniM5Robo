#include <Arduino.h>
#include <M5Unified.h>
#include <vector>
#include "LiDAR/LiDAR.hpp"
#include "config/config.hpp"
#include "utility/imu_util.hpp"
#include "utility/math_util.hpp"
#include "ota/ota.h"

LiDAR lidar(Serial1);

void setup(void)
{
  auto cfg = M5.config();
  M5.begin(cfg);
  // ディスプレイ設定
  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.setTextSize(2);
  Serial.begin(115200);
  // setupOTA();

  lidar.begin(32, 33);

  delay(10);

  Serial.printf("Start\n");
}

void loop()
{
  M5.update();
  lidar.update();
  float dt = (float)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  // M5.Display.startWrite();
  // M5.Display.setCursor(0, 20);
  // M5.Display.printf("ip:%s\n", WiFi.localIP().toString().c_str());
  // M5.Display.endWrite();
}
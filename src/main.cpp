#include <Arduino.h>
#include <M5Unified.h>
#include <SD.h>
#include <vector>
#include <gob_unifiedButton.hpp>
#include "LiDAR/LiDAR.hpp"
#include "config/config.hpp"
#include "utility/imu_util.hpp"
#include "utility/math_util.hpp"
#include "utility/dynamixel_utils.hpp"
#include <MadgwickAHRS.h>
// micro-ROS
#include <time.h>
#include "ros/wifi.h"
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <nav_msgs/msg/odometry.h>
// rtos
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#define ESP32_RTOS
#include "ota/ota.h"

rcl_publisher_t imu_pub;
rcl_publisher_t laser_scan_pub;
rcl_publisher_t odom_pub;
rcl_subscription_t cmd_vel_sub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t rcl_timer;

geometry_msgs__msg__Twist cmd_vel_msg;
sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;

goblib::UnifiedButton unifiedButton;

common_utils::rpy_t est_rpy;
float odom_yaw = 0.0f;
float gx, gy, gz;
float ax, ay, az;

LiDAR lidar(LIDAR_SERIAL);

bool dynamixel_init = false;
void main_task(void *arg);
void control_task(void *arg);
void odom_task(void *arg);
void sensor_task(void *arg);
void lidar_task(void *arg);
void publish_task(void *arg);

// callback function
void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  cmd_vel_msg = *msg;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  time_t now = time(NULL);
  rcl_publish(&imu_pub, &imu_msg, NULL);
  rcl_publish(&odom_pub, &odom_msg, NULL);
  rcl_publish(&laser_scan_pub, &lidar.laser_scan_msg, NULL);
}

bool odom_reset = false;
bool imu_reset = false;

void setup(void)
{
  auto cfg = M5.config();
  M5.begin(cfg);
  // ディスプレイ設定
  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.setTextSize(2);
  // button設定
  LovyanGFX *gfx = &M5.Display;
  unifiedButton.begin(&M5.Display);
  int32_t w{gfx->width() / 3};
  int32_t h{24};
  int32_t left{(gfx->width() - w * 3) / 2};
  int32_t top{gfx->height() - h};
  auto btnA = unifiedButton.getButtonA();
  assert(btnA);
  btnA->initButtonUL(unifiedButton.gfx(), left + w * 0, top, w, h, TFT_DARKGRAY, TFT_BLACK, TFT_DARKGRAY, "<");
  auto btnB = unifiedButton.getButtonB();
  assert(btnB);
  btnB->initButtonUL(unifiedButton.gfx(), left + w * 1, top, w, h, TFT_DARKGRAY, TFT_BLACK, TFT_DARKGRAY, "o");
  auto btnC = unifiedButton.getButtonC();
  assert(btnC);
  btnC->initButtonUL(unifiedButton.gfx(), left + w * 2, top, w, h, TFT_DARKGRAY, TFT_BLACK, TFT_DARKGRAY, ">");
  // setup ota
  Serial.begin(115200);
  setupOTA();

  M5.Display.setCursor(0, 0);
  M5.Display.printf("Connecting to WiFi...\n");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
  M5.Display.printf("ip:%s\n", WiFi.localIP().toString().c_str());
  // set_microros_serial_transports(Serial);
  IPAddress agent_ip;
  agent_ip.fromString(MICROROS_AGENT_IP);
  uint16_t agent_port = MICROROS_AGENT_PORT;
  set_microros_wifi_transports(agent_ip, agent_port);
  // set_microros_wifi_transports(MICROROS_WIFI_SSID, MICROROS_WIFI_PASSWORD, agent_ip, agent_port);
  delay(2000);
  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.printf("Time set\n");
  // NTPサーバーに接続して時間を調整する
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  delay(3000);
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "mini_m5_robo_node", "", &support);
  // Publisher
  rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu");
  rclc_publisher_init_default(&laser_scan_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan");
  rclc_publisher_init_default(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");
  // Subscriber
  rclc_subscription_init_best_effort(
      &cmd_vel_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel");

  // Timer
  rclc_timer_init_default(&rcl_timer, &support, RCL_MS_TO_NS(150), timer_callback);

  // Executor
  int callback_size = 2;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, callback_size, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &subscription_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &rcl_timer);

  // Task
  xTaskCreatePinnedToCore(main_task, "main task", 10000, NULL, 10, NULL, 1);
  xTaskCreatePinnedToCore(control_task, "control task", 4048, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(sensor_task, "sensor task", 4048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(lidar_task, "lidar task", 10000, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(odom_task, "odom task", 4048, NULL, 4, NULL, 0);

  Serial.printf("Start\n");
  avatar.setBatteryIcon(true);
  avatar.init();
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
}

void main_task(void *arg)
{
  display_mode = DisplayMode::AVATAR;
  DisplayMode last_display_mode = display_mode;
  bool avater_started = true;
  bool ota_start_flag = false;
  common_utils::rpy_t deg_rpy;
  while (true)
  {
    M5.update();
    unifiedButton.update();
    // reset
    if (reset_flag)
    {
      M5.Display.fillScreen(BLACK);
      if (odom_reset && imu_reset)
      {
        display_mode = last_display_mode;
        M5.Display.setCursor(0, 0);
        M5.Display.printf("Reset complete\n");
        reset_flag = false;
        odom_reset = false;
        imu_reset = false;
        avater_started = false;
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    }
    if (ota_started)
    {
      if (!ota_start_flag)
      {
        ota_start_flag = true;
        display_mode = DisplayMode::OTA;
        avatar.suspend();
        M5.Display.fillScreen(BLACK);
      }
    }
    // button
    if (M5.BtnB.wasHold())
    {
      reset_flag = true;
      display_mode = DisplayMode::RESET;
      avater_started = false;
      avatar.suspend();
      M5.Display.fillScreen(BLACK);
    }
    if (M5.BtnA.wasPressed())
    {
      sift_display_mode();
      if (display_mode != DisplayMode::AVATAR)
      {
        avatar.suspend();
        avater_started = false;
      }
      M5.Display.fillScreen(BLACK);
    }
    else if (M5.BtnB.wasPressed())
    {
    }
    else if (M5.BtnC.wasPressed())
    {
      sift_display_mode(true);
      if (display_mode != DisplayMode::AVATAR)
      {
        avatar.suspend();
        avater_started = false;
      }
      M5.Display.fillScreen(BLACK);
    }
    // display
    switch (display_mode)
    {
    case DisplayMode::AVATAR:
      if (!avater_started)
      {
        avatar.resume();
        avater_started = true;
      }
      avatar.setBatteryStatus(M5.Power.isCharging(), M5.Power.getBatteryLevel());
      last_display_mode = display_mode;
      break;
    case DisplayMode::RESET:
      M5.Display.startWrite();
      M5.Display.setCursor(0, 0);
      M5.Display.printf("Resetting...\n");
      M5.Display.endWrite();
      vTaskDelay(pdMS_TO_TICKS(100));
      break;
    case DisplayMode::OTA:
      M5.Display.startWrite();
      M5.Display.setCursor(0, 0);
      M5.Display.printf("OTA Progress: %u%%\n", ota_progress);
      M5.Display.progressBar(0, M5.Display.height() / 2, M5.Display.width(), 20, ota_progress);
      if (!ota_error.empty())
      {
        M5.Display.printf("Error: %s\n", ota_error.c_str());
        display_mode = last_display_mode;
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      M5.Display.endWrite();
      break;
    case DisplayMode::INFO:
      deg_rpy = common_utils::to_degrees(est_rpy);
      M5.Display.startWrite();
      M5.Display.setCursor(0, 0);
      M5.Display.printf("ip:%s\n", WiFi.localIP().toString().c_str());
      M5.Display.printf("Battery: %d%% Chg %d\n", M5.Power.getBatteryLevel(), M5.Power.isCharging());
      M5.Display.printf("v:%.2f, w:%.2f\n", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
      M5.Display.printf("x:%.2f, y:%.2f, yaw:%.2f\n", odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_yaw);
      M5.Display.printf("r:%.2f, p:%.2f, y:%.2f\n", deg_rpy.roll, deg_rpy.pitch, deg_rpy.yaw);
      M5.Display.endWrite();
      last_display_mode = display_mode;
      break;
    case DisplayMode::LIDAR:
      last_display_mode = display_mode;
      M5.Display.fillScreen(BLACK);
      lidar.draw_pointcloud();
      break;
    default:
      break;
    }
    unifiedButton.draw(true);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void control_task(void *arg)
{
  move_ = std::make_shared<kinematics::TwoWheelsf>(WHEEL_RADIUS, WHEEL_D);
  // dynamixel 設定
  DXL_SERIAL.begin(57600, SERIAL_8N1, RX_SERVO, TX_SERVO);
  dxl = Dynamixel2Arduino(DXL_SERIAL);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // motor setup
  // wheel
  m_lw.begin(false, false);
  m_rw.begin(true, false);
  m_lw.set_velocity_limit(-MAX_RPM, MAX_RPM);
  m_rw.set_velocity_limit(-MAX_RPM, MAX_RPM);
  m_lw.on(false);
  m_rw.on(false);
  vTaskDelay(pdMS_TO_TICKS(10));
  m_lw.on(true);
  m_rw.on(true);
  dynamixel_init = true;
  while (true)
  {
    move_->move(cmd_vel_msg.linear.x, 0.0, cmd_vel_msg.angular.z);
    auto wheel_speeds = move_->get_wheel_speeds();
    m_lw.move(wheel_speeds[0] * common_utils::constants::RPS_TO_RPM);
    m_rw.move(wheel_speeds[1] * common_utils::constants::RPS_TO_RPM);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void odom_task(void *arg)
{
  using namespace common_utils;
  while (!dynamixel_init)
  {
    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for dynamixel initialization
  }
  float RPM_TO_RADPS = constants::RPM_TO_RPS * constants::RPS_TO_RADPS;
  uint32_t odom_timer = micros();
  float x = 0.0;
  float y = 0.0;
  odom_yaw = 0.0;
  odom_msg.header.frame_id.data = (char *)"odom";
  odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
  odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;
  const float TWO_WHEEL_D = WHEEL_D * 2.0;                // 車輪間距離の2倍
  const float HALF_WHEEL_R = WHEEL_RADIUS / 2.0;          // 車輪半径の半分
  const float ANGULAR_CONST = WHEEL_RADIUS / TWO_WHEEL_D; // 角速度計算用定数
  while (true)
  {
    float odom_dt = (float)(micros() - odom_timer) / 1000000; // Calculate delta time
    odom_timer = micros();
    if (reset_flag)
    {
      x = 0.0;
      y = 0.0;
      odom_yaw = 0.0;
      odom_reset = true;
    }
    float l_rpm = m_lw.get_velocity();
    float r_rpm = m_rw.get_velocity();
    float w_l = l_rpm * RPM_TO_RADPS;
    float w_r = r_rpm * RPM_TO_RADPS;
    float vx = HALF_WHEEL_R * (w_l + w_r);
    float angular = ANGULAR_CONST * (w_r - w_l);
    x += vx * cos(odom_yaw) * odom_dt;
    y += vx * sin(odom_yaw) * odom_dt;
    odom_yaw += angular * odom_dt;
    odom_msg.header.stamp.sec = (int32_t)time(NULL);
    odom_msg.header.stamp.nanosec = (uint32_t)(micros() % 1000000);
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.z = sin(odom_yaw / 2.0);
    odom_msg.pose.pose.orientation.w = cos(odom_yaw / 2.0);
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.angular.z = angular;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

float sensor_dt = 0.;
void sensor_task(void *arg)
{
  using namespace common_utils;
  uint32_t sensor_timer = micros();
  imu_msg.header.frame_id.data = (char *)"imu_frame";
  imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;
  Madgwick filter;
  std::shared_ptr<Madgwick> filter_ptr = std::make_shared<Madgwick>(filter);
  while (1)
  {
    sensor_dt = (float)(micros() - sensor_timer) / 1000000; // Calculate delta time
    sensor_timer = micros();
    if (claib_flag)
    {
      gyro_caliblation();
      return;
    }
    if (reset_flag)
    {
      filter_ptr = std::make_shared<Madgwick>(filter);
      imu_reset = true;
    }
    // IMU
    float raw_ax, raw_ay, raw_az;
    float raw_gx, raw_gy, raw_gz;
    M5.Imu.getAccel(&raw_ax, &raw_ay, &raw_az);
    M5.Imu.getGyro(&raw_gx, &raw_gy, &raw_gz);
    gx = (raw_gx - gyro_offset[0]);
    gy = (raw_gy - gyro_offset[1]);
    gz = (raw_gz - gyro_offset[2]);
    ax = raw_ax;
    ay = raw_ay;
    az = raw_az;
    if (approx_zero(gx, GYRO_MIN_VALUE))
      gx = 0.f;
    if (approx_zero(gy, GYRO_MIN_VALUE))
      gy = 0.f;
    if (approx_zero(gz, GYRO_MIN_VALUE))
      gz = 0.f;
    // 姿勢計算
    filter_ptr->begin(1.0 / sensor_dt);
    filter_ptr->updateIMU(gx, gy, gz, ax, ay, az);

    est_rpy.roll = filter_ptr->getRollRadians();
    est_rpy.pitch = filter_ptr->getPitchRadians();
    est_rpy.yaw = filter_ptr->getYawRadians();

    imu_msg.header.stamp.sec = (int32_t)time(NULL);
    imu_msg.header.stamp.nanosec = (uint32_t)(micros() % 1000000);
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;
    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;
    quat_t q = to_quat(est_rpy);
    imu_msg.orientation.x = q.x;
    imu_msg.orientation.y = q.y;
    imu_msg.orientation.z = q.z;
    imu_msg.orientation.w = q.w;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void lidar_task(void *arg)
{
  using namespace common_utils;
  lidar.begin(LIDAR_RX, LIDAR_TX);
  lidar.set_visualize(true);
  lidar.laser_scan_msg.header.frame_id.data = (char *)"laser_frame";
  lidar.laser_scan_msg.header.frame_id.size = strlen(lidar.laser_scan_msg.header.frame_id.data);
  lidar.laser_scan_msg.header.frame_id.capacity = lidar.laser_scan_msg.header.frame_id.size + 1;
  while (1)
  {
    lidar.update();
    lidar.laser_scan_msg.header.stamp.sec = (int32_t)time(NULL);
    lidar.laser_scan_msg.header.stamp.nanosec = (uint32_t)(micros() % 1000000);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
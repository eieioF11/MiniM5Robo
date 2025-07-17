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
// micro-ROS
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
// rcl_timer_t rcl_timer;

geometry_msgs__msg__Twist cmd_vel_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__LaserScan laser_scan_msg;
nav_msgs__msg__Odometry odom_msg;

common_utils::rpy_t est_rpy;
float gx, gy, gz;
float ax, ay, az;

LiDAR lidar(LIDAR_SERIAL);

bool dynamixel_init = false;
void control_task(void *parameter)
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
  vTaskDelay(pdMS_TO_TICKS(10));
  dynamixel_init = true;
  while (true)
  {
    move_->move(cmd_vel_msg.linear.x, 0.0, cmd_vel_msg.angular.z);
    auto wheel_speeds = move_->get_wheel_speeds();
    m_lw.move(wheel_speeds[0] * common_utils::constants::RPS_TO_RPM);
    m_rw.move(wheel_speeds[1] * common_utils::constants::RPS_TO_RPM);
    // m_lw.move(5.0);
    // m_rw.move(5.0);
    vTaskDelay(pdMS_TO_TICKS(20)); // Delay for 1 ms
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
  float yaw = 0.0;
  while (true)
  {
    float odom_dt = (float)(micros() - odom_timer) / 1000000; // Calculate delta time
    odom_timer = micros();
    float l_rpm = m_lw.get_velocity();
    float r_rpm = m_rw.get_velocity();
    float w_l = l_rpm * RPM_TO_RADPS;
    float w_r = r_rpm * RPM_TO_RADPS;
    float rx = WHEEL_RADIUS * (w_l + w_r) / 2.0;
    float angular = WHEEL_RADIUS * (w_r - w_l) / (2.0 * WHEEL_D);
    odom_msg.header.frame_id.data = (char *)"odom";
    odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
    odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;
    odom_msg.header.stamp.sec = (int32_t)time(NULL);
    odom_msg.header.stamp.nanosec = (uint32_t)(micros() % 1000000);
    odom_msg.pose.pose.position.x += rx * cos(yaw) * odom_dt;
    odom_msg.pose.pose.position.y += rx * sin(yaw) * odom_dt;
    yaw += angular * odom_dt;
    odom_msg.pose.pose.orientation.z = sin(yaw / 2.0);
    odom_msg.pose.pose.orientation.w = cos(yaw / 2.0);
    // rcl_publish(&odom_pub, &odom_msg, NULL);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

float sensor_dt = 0.;
void sensor_task(void *arg)
{
  using namespace common_utils;
  uint32_t sensor_timer = micros();
  while (1)
  {
    M5.update();
    sensor_dt = (float)(micros() - sensor_timer) / 1000000; // Calculate delta time
    sensor_timer = micros();
    if (claib_flag)
    {
      gyro_caliblation();
      return;
    }
    // IMU
    float raw_ax, raw_ay, raw_az;
    float raw_gx, raw_gy, raw_gz;
    M5.Imu.getAccel(&raw_ax, &raw_ay, &raw_az);
    M5.Imu.getGyro(&raw_gx, &raw_gy, &raw_gz);
    gx = (raw_gx - gyro_offset[0]) * DEG_TO_RAD;
    gy = (raw_gy - gyro_offset[1]) * DEG_TO_RAD;
    gz = (raw_gz - gyro_offset[2]) * DEG_TO_RAD;
    ax = raw_ax;
    ay = raw_ay;
    az = raw_az;
    // 姿勢計算
    // rpy_t a_rpy = acc_rpy(ax, ay, az);
    // rpy_t g_rpy = gyro_rpy(est_rpy, gx, gy, gz, sensor_dt);
    // est_rpy.yaw = g_rpy.yaw;
    // a_rpy.roll = normalize_angle(a_rpy.roll - HALF_PI);
    // est_rpy.roll = lpf_acc_x.filtering(a_rpy.roll);
    // est_rpy.pitch = lpf_acc_y.filtering(a_rpy.pitch);
    // est_rpy.roll = comp_filter_x.filtering(a_rpy.roll, g_rpy.roll);
    // est_rpy.pitch = comp_filter_y.filtering(a_rpy.pitch, g_rpy.pitch);
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;
    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;
    // imu_msg.orientation.x = 0.0;
    // imu_msg.orientation.y = 0.0;
    // imu_msg.orientation.z = sin(est_rpy.yaw / 2.0);
    // imu_msg.orientation.w = cos(est_rpy.yaw / 2.0);
    // if (approx_zero(gx, 0.005))
    //   gx = 0.f;
    rcl_publish(&imu_pub, &imu_msg, NULL);
    rcl_publish(&odom_pub, &odom_msg, NULL);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void lidar_task(void *arg)
{
  using namespace common_utils;
  lidar.begin(LIDAR_RX, LIDAR_TX);
  lidar.set_visualize(false);
  while (1)
  {
    if (lidar.update())
    {
      laser_scan_t scan = lidar.get_laser_scan();
      laser_scan_msg.header.frame_id.data = (char *)"laser_frame";
      laser_scan_msg.header.frame_id.size = strlen(laser_scan_msg.header.frame_id.data);
      laser_scan_msg.header.frame_id.capacity = laser_scan_msg.header.frame_id.size + 1;
      laser_scan_msg.header.stamp.sec = (int32_t)time(NULL);
      laser_scan_msg.header.stamp.nanosec = (uint32_t)(micros() % 1000000);
      laser_scan_msg.angle_min = scan.angle_min;
      laser_scan_msg.angle_max = scan.angle_max;
      laser_scan_msg.angle_increment = scan.angle_increment;
      laser_scan_msg.time_increment = scan.time_increment;
      laser_scan_msg.scan_time = scan.scan_time;
      laser_scan_msg.range_min = scan.range_min;
      laser_scan_msg.range_max = scan.range_max;
      laser_scan_msg.ranges.size = scan.ranges.size();
      laser_scan_msg.ranges.data = (float *)malloc(scan.ranges.size() * sizeof(float));
      laser_scan_msg.intensities.size = scan.intensities.size();
      laser_scan_msg.intensities.data = (float *)malloc(scan.intensities.size() * sizeof(float));
      for (size_t i = 0; i < scan.ranges.size(); i++)
      {
        laser_scan_msg.ranges.data[i] = scan.ranges[i];
        laser_scan_msg.intensities.data[i] = scan.intensities[i];
      }
      rcl_publish(&laser_scan_pub, &laser_scan_msg, NULL);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// サブスクライバーのコールバック関数
void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  cmd_vel_msg = *msg;
}

void setup(void)
{
  auto cfg = M5.config();
  M5.begin(cfg);
  // ディスプレイ設定
  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.setTextSize(2);
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

  // Executor
  int callback_size = 1;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, callback_size, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &subscription_callback, ON_NEW_DATA);

  xTaskCreatePinnedToCore(control_task, "control task", 4048, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(sensor_task, "sensor task", 4048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(lidar_task, "lidar task", 4048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(odom_task, "odom task", 4048, NULL, 4, NULL, 0);
  Serial.printf("Start\n");
  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.printf("Start\n");
  M5.Display.printf("ip:%s\n", WiFi.localIP().toString().c_str());
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
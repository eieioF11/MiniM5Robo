#include "LTR553Als/LTR553Als.hpp"
#include "LiDAR/LiDAR.hpp"
#include "config/config.hpp"
#include "config/wifi_info.hpp"
#include "utility/dynamixel_utils.hpp"
#include "utility/imu_util.hpp"
#include "utility/math_util.hpp"
#include <Arduino.h>
#include <M5Unified.h>
#include <MadgwickAHRS.h>
#include <SD.h>
#include <gob_unifiedButton.hpp>
#include <vector>

// micro-ROS
#include "ros/wifi.h"
#include <geometry_msgs/msg/twist.h>
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/compressed_image.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int16.h>
#include <time.h>

// rtos
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#define ESP32_RTOS
#include "camera/camera.hpp"
#include "ota/ota.h"

// --- イベントフラグの定義 ---
#define EVT_DYNAMIXEL_INIT_DONE (1 << 0)
#define EVT_RESET_REQ (1 << 1)
#define EVT_RESET_ODOM_DONE (1 << 2)
#define EVT_RESET_IMU_DONE (1 << 3)
#define EVT_RESET_DXL_DONE (1 << 4)
#define EVT_OTA_START (1 << 5)

// --- ロボットコンテキスト (グローバル変数の代替) ---
struct RobotContext {
  QueueHandle_t cmd_vel_queue;
  QueueHandle_t dxl_torque_queue;
  QueueHandle_t power_queue;
  EventGroupHandle_t system_events;
  SemaphoreHandle_t display_mutex;
  SemaphoreHandle_t ros_msg_mutex;

  float l_rpm;
  float r_rpm;
  float odom_yaw;
  common_utils::rpy_t est_rpy;
  bool claib_flag;
  DisplayMode display_mode;
};

RobotContext g_ctx;

// ROS 2関連
rcl_publisher_t imu_pub;
rcl_publisher_t laser_scan_pub;
rcl_publisher_t odom_pub;
rcl_publisher_t image_pub;
rcl_publisher_t als_ps_pub;
rcl_subscription_t cmd_vel_sub;
rcl_subscription_t dxl_torque_sub;
rcl_subscription_t power_sub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t rcl_timer;

geometry_msgs__msg__Twist cmd_vel_msg;
sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__CompressedImage image_msg;
std_msgs__msg__Bool dxl_torque_msg;
std_msgs__msg__Bool power_msg;
std_msgs__msg__UInt16 als_ps_msg;

goblib::UnifiedButton unifiedButton;
LiDAR::VI4300 lidar(LIDAR_SERIAL);
Camera::GC0308 camera;
proximity_sensor::LTR553Als ltr553als;

void main_task(void* arg);
void control_task(void* arg);
void odom_task(void* arg);
void sensor_task(void* arg);
void lidar_task(void* arg);

// --- コールバック関数 ---
void cmd_vel_sub_callback(const void* msgin) {
  const auto* msg = (const geometry_msgs__msg__Twist*)msgin;
  xQueueSendToBack(g_ctx.cmd_vel_queue, msg, 0);
}

void dxl_torque_sub_callback(const void* msgin) {
  const auto* msg = (const std_msgs__msg__Bool*)msgin;
  xQueueSendToBack(g_ctx.dxl_torque_queue, &(msg->data), 0);
}

void power_sub_callback(const void* msgin) {
  const auto* msg = (const std_msgs__msg__Bool*)msgin;
  xQueueSendToBack(g_ctx.power_queue, &(msg->data), 0);
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  // if (xSemaphoreTake(g_ctx.ros_msg_mutex, pdMS_TO_TICKS(0)) == pdTRUE) {
  rcl_publish(&imu_pub, &imu_msg, NULL);
  rcl_publish(&odom_pub, &odom_msg, NULL);
  rcl_publish(&laser_scan_pub, &lidar.laser_scan_msg, NULL);
  rcl_publish(&image_pub, &image_msg, NULL);
  rcl_publish(&als_ps_pub, &als_ps_msg, NULL);
  //   xSemaphoreGive(g_ctx.ros_msg_mutex);
  // }
}

// --- Setup ---
void setup() {
  auto cfg         = M5.config();
  cfg.output_power = false;
  M5.begin(cfg);
  Serial.begin(115200);

  g_ctx.cmd_vel_queue    = xQueueCreate(5, sizeof(geometry_msgs__msg__Twist));
  g_ctx.dxl_torque_queue = xQueueCreate(5, sizeof(bool));
  g_ctx.power_queue      = xQueueCreate(5, sizeof(bool));
  g_ctx.system_events    = xEventGroupCreate();
  g_ctx.display_mutex    = xSemaphoreCreateMutex();
  g_ctx.ros_msg_mutex    = xSemaphoreCreateMutex();
  g_ctx.display_mode     = DisplayMode::AVATAR;
  g_ctx.claib_flag       = false;

  M5.Display.setFont(&fonts::efontJA_10);
  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.setTextSize(2);

  LovyanGFX* gfx = &M5.Display;
  unifiedButton.begin(&M5.Display);
  int32_t w{gfx->width() / 3};
  int32_t h{32};
  int32_t left{(gfx->width() - w * 3) / 2};
  int32_t top{gfx->height() - h};
  unifiedButton.getButtonA()->initButtonUL(unifiedButton.gfx(), left + w * 0, top, w, h, TFT_DARKGRAY, TFT_BLACK, TFT_DARKGRAY, "<");
  unifiedButton.getButtonB()->initButtonUL(unifiedButton.gfx(), left + w * 1, top, w, h, TFT_DARKGRAY, TFT_BLACK, TFT_DARKGRAY, "o");
  unifiedButton.getButtonC()->initButtonUL(unifiedButton.gfx(), left + w * 2, top, w, h, TFT_DARKGRAY, TFT_BLACK, TFT_DARKGRAY, ">");

  M5.Display.printf("aw9523 Init\n");
  aw9523_begin();
  M5.Display.printf("SD card %d\n", sd_exist());

  M5.Display.printf("Setup OTA\n");
  const auto [success, ssid, password, cfg_agent_ip, cfg_agent_port] = get_config();
  if (success)
    setupOTA(ssid.c_str(), password.c_str());
  else
    setupOTA(WIFI_SSID, WIFI_PASSWORD);

  M5.Display.printf("Connecting to WiFi...\n");
  while (WiFi.status() != WL_CONNECTED)
    delay(500);
  M5.Display.printf("ip:%s\n", WiFi.localIP().toString().c_str());

  IPAddress agent_ip;
  int16_t agent_port = MICROROS_AGENT_PORT;
  if (success) {
    agent_ip.fromString(cfg_agent_ip.c_str());
    agent_port = cfg_agent_port;
  } else {
    agent_ip.fromString(MICROROS_AGENT_IP);
  }
  set_microros_wifi_transports(agent_ip, agent_port);
  delay(2000);

  M5.Display.printf("Time set\n");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  delay(3000);

  M5.Display.printf("Camera Init\n");
  camera.begin();

  M5.Display.printf("micro-ROS Init\n");
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "mini_m5_robo_node", "", &support);

  rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu");
  rclc_publisher_init_default(&laser_scan_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan");
  rclc_publisher_init_default(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");
  rclc_publisher_init_default(&image_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage), "/camera/compressed_image");
  rclc_publisher_init_default(&als_ps_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16), "/als_ps");

  rclc_subscription_init_best_effort(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
  rclc_subscription_init_best_effort(&dxl_torque_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/dxl_torque_enable");
  rclc_subscription_init_best_effort(&power_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/power_enable");

  rclc_timer_init_default(&rcl_timer, &support, RCL_MS_TO_NS(100), timer_callback);
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, 4, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_sub_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &dxl_torque_sub, &dxl_torque_msg, &dxl_torque_sub_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &power_sub, &power_msg, &power_sub_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &rcl_timer);

  // 【重要】タスク優先度の見直し
  // Core 0 (ハードウェア制御)
  xTaskCreatePinnedToCore(control_task, "control_task", 4048, &g_ctx, 2, NULL, 0);
  xTaskCreatePinnedToCore(lidar_task, "lidar_task", 10000, &g_ctx, 3, NULL, 0);
  xTaskCreatePinnedToCore(odom_task, "odom_task", 4048, &g_ctx, 1, NULL, 0);

  // Core 1 (システム・UI)
  xTaskCreatePinnedToCore(main_task, "main_task", 10000, &g_ctx, 2, NULL, 1);
  xTaskCreatePinnedToCore(sensor_task, "sensor_task", 10000, &g_ctx, 1, NULL, 1);

  M5.Display.fillScreen(BLACK);
  avatar.setBatteryIcon(true);
  avatar.init();
  Serial.printf("Start\n");
  M5.Speaker.tone(900, 100);
}

void loop() { rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)); }

// --- メインタスク ---
void main_task(void* arg) {
  auto* ctx                     = static_cast<RobotContext*>(arg);
  DisplayMode last_display_mode = ctx->display_mode;
  bool avatar_started           = true;
  bool ota_start_flag           = false;
  common_utils::rpy_t deg_rpy;
  bool power_state      = false;
  bool dxl_torque_state = true;

  while (true) {
    unifiedButton.update();
    EventBits_t events = xEventGroupGetBits(ctx->system_events);

    if (events & EVT_RESET_REQ) {
      if ((events & EVT_RESET_ODOM_DONE) && (events & EVT_RESET_IMU_DONE) && (events & EVT_RESET_DXL_DONE)) {
        if (xSemaphoreTake(ctx->display_mutex, portMAX_DELAY) == pdTRUE) {
          M5.Display.fillScreen(BLACK);
          ctx->display_mode = last_display_mode;
          M5.Display.setCursor(0, 0);
          M5.Display.printf("Reset complete\n");
          xSemaphoreGive(ctx->display_mutex);
        }
        xEventGroupClearBits(ctx->system_events, EVT_RESET_REQ | EVT_RESET_ODOM_DONE | EVT_RESET_IMU_DONE | EVT_RESET_DXL_DONE);
        avatar_started = false;
        M5.Speaker.tone(1000, 500);
      }
    }

    if (events & EVT_OTA_START) {
      if (!ota_start_flag) {
        ota_start_flag    = true;
        ctx->display_mode = DisplayMode::OTA;
        avatar.suspend();
        if (xSemaphoreTake(ctx->display_mutex, portMAX_DELAY) == pdTRUE) {
          M5.Display.fillScreen(BLACK);
          xSemaphoreGive(ctx->display_mutex);
        }
      }
    }

    if (M5.BtnA.wasHold()) {
      if (xSemaphoreTake(ctx->display_mutex, portMAX_DELAY) == pdTRUE) {
        M5.Display.fillScreen(BLACK);
        ctx->display_mode = last_display_mode;
        dxl_torque_state  = !dxl_torque_state;
        xQueueSendToBack(ctx->dxl_torque_queue, &dxl_torque_state, 0);
        M5.Display.startWrite();
        M5.Display.setCursor(0, 0);
        M5.Display.printf("Dynamixel torque %s\n", dxl_torque_state ? "ON" : "OFF");
        M5.Display.endWrite();
        xSemaphoreGive(ctx->display_mutex);
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
    } else if (M5.BtnA.wasReleased()) {
      sift_display_mode();
      ctx->display_mode = display_mode;
      if (ctx->display_mode != DisplayMode::AVATAR) {
        avatar.suspend();
        avatar_started = false;
      }
      if (xSemaphoreTake(ctx->display_mutex, portMAX_DELAY) == pdTRUE) {
        M5.Display.fillScreen(BLACK);
        xSemaphoreGive(ctx->display_mutex);
      }
      M5.Speaker.tone(1000, 50);
    }

    if (M5.BtnB.wasHold()) {
      xEventGroupSetBits(ctx->system_events, EVT_RESET_REQ);
      ctx->display_mode = DisplayMode::RESET;
      avatar_started    = false;
      avatar.suspend();
      if (xSemaphoreTake(ctx->display_mutex, portMAX_DELAY) == pdTRUE) {
        M5.Display.fillScreen(BLACK);
        xSemaphoreGive(ctx->display_mutex);
      }
    } else if (M5.BtnB.wasReleased()) {
      ctx->display_mode = DisplayMode::AVATAR;
      if (xSemaphoreTake(ctx->display_mutex, portMAX_DELAY) == pdTRUE) {
        M5.Display.fillScreen(BLACK);
        xSemaphoreGive(ctx->display_mutex);
      }
      M5.Speaker.tone(1000, 50);
    }

    if (M5.BtnC.wasReleased()) {
      sift_display_mode(true);
      ctx->display_mode = display_mode;
      if (ctx->display_mode != DisplayMode::AVATAR) {
        avatar.suspend();
        avatar_started = false;
      }
      if (xSemaphoreTake(ctx->display_mutex, portMAX_DELAY) == pdTRUE) {
        M5.Display.fillScreen(BLACK);
        xSemaphoreGive(ctx->display_mutex);
      }
      M5.Speaker.tone(1000, 50);
    }

    if (xQueueReceive(ctx->power_queue, &power_state, 0) == pdTRUE) {
      M5.Power.setExtOutput(power_state);
      Serial.printf("Power %s\n", power_state ? "ON" : "OFF");
    }

    auto [als_ch0, als_ch1, ps_value, lux] = ltr553als.getValues();

    if (xSemaphoreTake(ctx->display_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      switch (ctx->display_mode) {
        case DisplayMode::AVATAR:
          if (!avatar_started) {
            avatar.resume();
            avatar_started = true;
          }
          avatar.setBatteryStatus(M5.Power.isCharging(), M5.Power.getBatteryLevel());
          last_display_mode = ctx->display_mode;
          break;
        case DisplayMode::RESET:
          M5.Display.startWrite();
          M5.Display.setCursor(0, 0);
          M5.Display.printf("Resetting...\n");
          M5.Display.endWrite();
          break;
        case DisplayMode::OTA:
          M5.Display.startWrite();
          M5.Display.setCursor(0, 0);
          M5.Display.printf("OTA Progress: %u%%\n", ota_progress);
          M5.Display.progressBar(0, M5.Display.height() / 2, M5.Display.width(), 20, ota_progress);
          M5.Display.endWrite();
          break;
        case DisplayMode::INFO:
          deg_rpy = common_utils::to_degrees(ctx->est_rpy);
          M5.Display.startWrite();
          M5.Display.setCursor(0, 0);
          M5.Display.printf("ip:%s\n", WiFi.localIP().toString().c_str());
          M5.Display.printf("Battery: %d%% Chg %d\n", M5.Power.getBatteryLevel(), M5.Power.isCharging());
          M5.Display.printf("dxl torque: %s\n", dxl_torque_state ? "ON" : "OFF");
          M5.Display.printf("Power %s\n", power_state ? "ON" : "OFF");
          M5.Display.printf("x:%.2f, y:%.2f, yaw:%.2f\n", odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, ctx->odom_yaw);
          M5.Display.printf("r:%.2f, p:%.2f, y:%.2f\n", deg_rpy.roll, deg_rpy.pitch, deg_rpy.yaw);
          M5.Display.printf("ALS CH0: %d, CH1: %d\n", als_ch0, als_ch1);
          M5.Display.printf("ALS PS: %d, Lux: %.2f\n", ps_value, lux);

          if (ps_value > 600)
            M5.Speaker.tone(ps_value, 10);
          else
            M5.Speaker.stop();

          M5.Display.endWrite();
          last_display_mode = ctx->display_mode;
          break;
        case DisplayMode::CAMERA:
          last_display_mode = ctx->display_mode;
          break;
        case DisplayMode::LIDAR:
          if (last_display_mode != ctx->display_mode) {
            M5.Display.fillScreen(BLACK);
          }
          last_display_mode = ctx->display_mode;
          lidar.draw_pointcloud();
          break;
        default:
          break;
      }
      unifiedButton.draw(true);
      xSemaphoreGive(ctx->display_mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

// --- コントロールタスク ---
void control_task(void* arg) {
  auto* ctx = static_cast<RobotContext*>(arg);
  move_     = std::make_shared<kinematics::TwoWheelsf>(WHEEL_RADIUS, WHEEL_D);

  DXL_SERIAL.begin(57600, SERIAL_8N1, RX_SERVO, TX_SERVO);
  dxl = Dynamixel2Arduino(DXL_SERIAL);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  m_lw.begin(false, false);
  m_rw.begin(true, false);
  m_lw.set_velocity_limit(-MAX_RPM, MAX_RPM);
  m_rw.set_velocity_limit(-MAX_RPM, MAX_RPM);
  m_lw.on(false);
  m_rw.on(false);
  vTaskDelay(pdMS_TO_TICKS(10));
  m_lw.on(true);
  m_rw.on(true);

  xEventGroupSetBits(ctx->system_events, EVT_DYNAMIXEL_INIT_DONE);

  // 【重要】ゴミデータによる暴走を防ぐためゼロ初期化
  geometry_msgs__msg__Twist local_cmd;
  memset(&local_cmd, 0, sizeof(geometry_msgs__msg__Twist));
  bool local_torque;

  while (true) {
    if (xEventGroupGetBits(ctx->system_events) & EVT_RESET_REQ) {
      Serial.printf("Dynamixel reset\n");
      m_lw.begin(false, false);
      m_rw.begin(true, false);
      xEventGroupSetBits(ctx->system_events, EVT_RESET_DXL_DONE);
    }

    if (xQueueReceive(ctx->dxl_torque_queue, &local_torque, 0) == pdTRUE) {
      m_lw.on(local_torque);
      m_rw.on(local_torque);
      Serial.printf("Dynamixel torque %s\n", local_torque ? "ON" : "OFF");
    }

    // ROSからの新しい指示があれば上書き。無ければ前回の速度を維持する。
    xQueueReceive(ctx->cmd_vel_queue, &local_cmd, 0);
    move_->move(local_cmd.linear.x, 0.0, local_cmd.angular.z);

    auto wheel_speeds = move_->get_wheel_speeds();
    m_lw.move(wheel_speeds[0] * common_utils::constants::RPS_TO_RPM);
    m_rw.move(wheel_speeds[1] * common_utils::constants::RPS_TO_RPM);

    ctx->l_rpm = m_lw.get_velocity();
    ctx->r_rpm = m_rw.get_velocity();

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// --- オドメトリタスク ---
void odom_task(void* arg) {
  auto* ctx = static_cast<RobotContext*>(arg);
  using namespace common_utils;

  xEventGroupWaitBits(ctx->system_events, EVT_DYNAMIXEL_INIT_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

  float RPM_TO_RADPS  = constants::RPM_TO_RPS * constants::RPS_TO_RADPS;
  uint32_t odom_timer = micros();
  float x = 0.0, y = 0.0;
  ctx->odom_yaw = 0.0;

  odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
  odom_msg.child_frame_id  = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_link");

  const float TWO_WHEEL_D   = WHEEL_D * 2.0;
  const float HALF_WHEEL_R  = WHEEL_RADIUS / 2.0;
  const float ANGULAR_CONST = WHEEL_RADIUS / TWO_WHEEL_D;

  while (true) {
    if (xEventGroupGetBits(ctx->system_events) & EVT_RESET_REQ) {
      x             = 0.0;
      y             = 0.0;
      ctx->odom_yaw = 0.0;
      xEventGroupSetBits(ctx->system_events, EVT_RESET_ODOM_DONE);
    }

    float odom_dt = (float)(micros() - odom_timer) / 1000000;
    odom_timer    = micros();

    float w_l     = ctx->l_rpm * RPM_TO_RADPS;
    float w_r     = ctx->r_rpm * RPM_TO_RADPS;
    float vx      = HALF_WHEEL_R * (w_l + w_r);
    float angular = ANGULAR_CONST * (w_r - w_l);

    x += vx * cos(ctx->odom_yaw) * odom_dt;
    y += vx * sin(ctx->odom_yaw) * odom_dt;
    ctx->odom_yaw += angular * odom_dt;

    // if (xSemaphoreTake(ctx->ros_msg_mutex, pdMS_TO_TICKS(0)) == pdTRUE) {
    odom_msg.header.stamp.sec        = (int32_t)time(NULL);
    odom_msg.header.stamp.nanosec    = (uint32_t)(micros() % 1000000);
    odom_msg.pose.pose.position.x    = x;
    odom_msg.pose.pose.position.y    = y;
    odom_msg.pose.pose.position.z    = 0.0;
    odom_msg.pose.pose.orientation.z = sin(ctx->odom_yaw / 2.0);
    odom_msg.pose.pose.orientation.w = cos(ctx->odom_yaw / 2.0);
    odom_msg.twist.twist.linear.x    = vx;
    odom_msg.twist.twist.angular.z   = angular;
    xSemaphoreGive(ctx->ros_msg_mutex);
    // }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// --- センサタスク ---
void high_rate_sensor_task(float dt, std::shared_ptr<Madgwick> filter_ptr, RobotContext* ctx) {
  using namespace common_utils;
  ltr553als.update();
  auto [als_ch0, als_ch1, ps_value, lux] = ltr553als.getValues();

  float raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
  M5.Imu.getAccel(&raw_ax, &raw_ay, &raw_az);
  M5.Imu.getGyro(&raw_gx, &raw_gy, &raw_gz);

  float gx = (raw_gx - gyro_offset[0]);
  float gy = (raw_gy - gyro_offset[1]);
  float gz = (raw_gz - gyro_offset[2]);
  if (approx_zero(gx, GYRO_MIN_VALUE)) gx = 0.f;
  if (approx_zero(gy, GYRO_MIN_VALUE)) gy = 0.f;
  if (approx_zero(gz, GYRO_MIN_VALUE)) gz = 0.f;

  filter_ptr->begin(1.0 / dt);
  filter_ptr->updateIMU(gx, gy, gz, raw_ax, raw_ay, raw_az);

  ctx->est_rpy.roll  = filter_ptr->getRollRadians();
  ctx->est_rpy.pitch = filter_ptr->getPitchRadians();
  ctx->est_rpy.yaw   = filter_ptr->getYawRadians();
  quat_t q           = to_quat(ctx->est_rpy);

  if (xSemaphoreTake(ctx->ros_msg_mutex, pdMS_TO_TICKS(0)) == pdTRUE) {
    als_ps_msg.data               = ps_value;
    imu_msg.header.stamp.sec      = (int32_t)time(NULL);
    imu_msg.header.stamp.nanosec  = (uint32_t)(micros() % 1000000);
    imu_msg.linear_acceleration.x = raw_ax;
    imu_msg.linear_acceleration.y = raw_ay;
    imu_msg.linear_acceleration.z = raw_az;
    imu_msg.angular_velocity.x    = gx;
    imu_msg.angular_velocity.y    = gy;
    imu_msg.angular_velocity.z    = gz;
    imu_msg.orientation.x         = q.x;
    imu_msg.orientation.y         = q.y;
    imu_msg.orientation.z         = q.z;
    imu_msg.orientation.w         = q.w;
    xSemaphoreGive(ctx->ros_msg_mutex);
  }
}

void low_rate_sensor_task(float dt, RobotContext* ctx) {
  camera.capture();
  bool jpeg_converted = camera.calcJpgFrameBuffer(JEPEG_QUALITY);

  if (ctx->display_mode == DisplayMode::CAMERA) {
    if (xSemaphoreTake(ctx->display_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      camera.draw_jpg();
      xSemaphoreGive(ctx->display_mutex);
    }
  }

  camera.returnFrameBuffer();
  camera_fb_t* jpg_fb = camera.getJpgFrameBuffer();

  if (jpeg_converted && jpg_fb->buf != NULL) {
    // if (xSemaphoreTake(ctx->ros_msg_mutex, pdMS_TO_TICKS(0)) == pdTRUE) {
    image_msg.header.stamp.sec     = (int32_t)time(NULL);
    image_msg.header.stamp.nanosec = (uint32_t)(micros() % 1000000);
    if (jpg_fb->len <= image_msg.data.capacity) {
      image_msg.data.size = jpg_fb->len;
      memcpy(image_msg.data.data, jpg_fb->buf, jpg_fb->len);
      image_msg.format = micro_ros_string_utilities_set(image_msg.format, "jpeg");
    }
    //   xSemaphoreGive(ctx->ros_msg_mutex);
    // }
    camera.returnJpgFrameBuffer();
  }
}

void sensor_task(void* arg) {
  auto* ctx = static_cast<RobotContext*>(arg);
  ltr553als.begin();
  imu_msg.header.frame_id   = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_frame");
  image_msg.header.frame_id = micro_ros_string_utilities_set(image_msg.header.frame_id, "rgb_frame");

  static micro_ros_utilities_memory_conf_t conf = {};
  conf.max_string_capacity                      = 50;
  conf.max_ros2_type_sequence_capacity          = 5;
  conf.max_basic_type_sequence_capacity         = 5;
  micro_ros_utilities_memory_rule_t rules[]     = {{"header.frame_id", 30}, {"format", 4}, {"data", 20000}};
  conf.rules                                    = rules;
  conf.n_rules                                  = sizeof(rules) / sizeof(rules[0]);
  micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage), &image_msg, conf);

  Madgwick filter;
  std::shared_ptr<Madgwick> filter_ptr = std::make_shared<Madgwick>(filter);
  uint32_t high_time_start             = micros();
  uint32_t low_time_start              = micros();

  while (1) {
    M5.update();
    if (ctx->claib_flag) {
      gyro_caliblation();
      return;
    }

    if (xEventGroupGetBits(ctx->system_events) & EVT_RESET_REQ) {
      filter_ptr = std::make_shared<Madgwick>(filter);
      xEventGroupSetBits(ctx->system_events, EVT_RESET_IMU_DONE);
    }

    float high_rate_dt = (float)(micros() - high_time_start) / 1000000;
    high_time_start    = micros();
    high_rate_sensor_task(high_rate_dt, filter_ptr, ctx);

    float low_rate_dt = (float)(micros() - low_time_start) / 1000000;
    if (low_rate_dt >= 0.01) {
      low_rate_sensor_task(low_rate_dt, ctx);
      low_time_start = micros();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// --- LiDARタスク ---
void lidar_task(void* arg) {
  auto* ctx = static_cast<RobotContext*>(arg);
  lidar.begin(LIDAR_RX, LIDAR_TX, "laser_frame");

  while (1) {
    // 【重要】taskYIELD() を削除し、ハードウェアバッファにあるデータを一気に読み切る
    while (lidar.update()) {
    }

    // if (xSemaphoreTake(ctx->ros_msg_mutex, pdMS_TO_TICKS(0)) == pdTRUE) {
    lidar.laser_scan_msg.header.stamp.sec     = (int32_t)time(NULL);
    lidar.laser_scan_msg.header.stamp.nanosec = (uint32_t)(micros() % 1000000);
    //   xSemaphoreGive(ctx->ros_msg_mutex);
    // }

    // 【重要】ここで確実にブロック状態を作り、優先度の低いタスクへCPUを譲る
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
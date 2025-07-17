#pragma once
#include <Dynamixel2Arduino.h>
#include <Arduino.h>
#include "utility/math_util.hpp"

Dynamixel2Arduino dxl;

struct dxl_status_t
{
  float position; // [rad]
  float velocity; // [rpm]
  float current;  // [A]
};

// OP_CURRENT
// OP_VELOCITY
// OP_POSITION
// OP_EXTENDED_POSITION
// OP_CURRENT_BASED_POSITION
// OP_PWM

class DXLMotor
{
private:
  uint8_t id_;
  uint8_t mode_;
  bool reverse_;
  float bias_;
  float max_pos_ = TWO_PI;
  float min_pos_ = -TWO_PI;
  float min_vel_ = -100000.0;
  float max_vel_ = 100000.0;

  void dxl_on(uint8_t id, bool sw)
  {
    if (sw)
    {
      dxl.ledOn(id);
      dxl.torqueOn(id);
    }
    else
    {
      dxl.ledOff(id);
      dxl.torqueOff(id);
    }
  }
  void set_dxl(uint8_t id, uint8_t mode)
  {
    dxl.ping(id);
    dxl.torqueOff(id);
    dxl.ledOff(id);
    dxl_on(id, false);
    dxl.setOperatingMode(id, mode);
    dxl_on(id, true);
    delay(10);
  }
  void move_vel(uint8_t id, float rpm)
  {
    if (rpm > max_vel_)
      rpm = max_vel_;
    if (rpm < min_vel_)
      rpm = min_vel_;
    dxl.setGoalVelocity(id, rpm, UNIT_RPM);
  }
  void move_current(uint8_t id, float current) { dxl.setGoalCurrent(id, current, UNIT_MILLI_AMPERE); }
  void move_pos(uint8_t id, float rad) { dxl.setGoalPosition(id, rad * RAD_TO_DEG, UNIT_DEGREE); }

public:
  DXLMotor(uint8_t id, uint8_t mode) : id_(id), mode_(mode) {}
  void begin(bool reverse = false, bool set_pos = false)
  {
    reverse_ = reverse;
    set_dxl(id_, mode_);
    dxl_on(id_, true);
    bias_ = 0;
    if (set_pos)
      set_position();
  }
  void set_position(float pos = 0.0, float range = HALF_PI)
  {
    dxl_status_t status = get_status();
    if (std::abs(status.position) < pos + range)
      bias_ = 0;
    else
      bias_ = TWO_PI;
  }
  void set_velocity_limit(float min, float max)
  {
    min_vel_ = min;
    max_vel_ = max;
  }
  void set_limit(float min_pos, float max_pos)
  {
    if (reverse_)
    {
      min_pos_ = -max_pos;
      max_pos_ = -min_pos;
    }
    else
    {
      min_pos_ = min_pos;
      max_pos_ = max_pos;
    }
  }
  void on(bool sw) { dxl_on(id_, sw); }
  void move(float val)
  {
    if (reverse_)
      val = -val;
    switch (mode_)
    {
    case OP_CURRENT:
      move_current(id_, val);
      break;
    case OP_VELOCITY:
      move_vel(id_, val);
      break;
    case OP_POSITION:
    case OP_EXTENDED_POSITION:
    case OP_CURRENT_BASED_POSITION:
      if (val > max_pos_)
        val = max_pos_;
      if (val < min_pos_)
        val = min_pos_;
      float angle = val + bias_;
      move_pos(id_, angle);
      break;
    }
  }
  float get_velocity()
  {
    float velocity = dxl.getPresentVelocity(id_, UNIT_RPM);
    if (reverse_)
      return -velocity;
    return velocity;
  }
  dxl_status_t get_status()
  {
    dxl_status_t status;
    status.position = dxl.getPresentPosition(id_, UNIT_DEGREE) * DEG_TO_RAD - bias_;
    status.velocity = dxl.getPresentVelocity(id_, UNIT_RPM);
    status.current = dxl.getPresentCurrent(id_, UNIT_MILLI_AMPERE) * 0.001;
    if (reverse_)
    {
      status.position = -status.position;
      status.velocity = -status.velocity;
      status.current = -status.current;
    }
    return status;
  }
  dxl_status_t get_raw_status()
  {
    dxl_status_t status;
    status.position = dxl.getPresentPosition(id_, UNIT_DEGREE);
    status.velocity = dxl.getPresentVelocity(id_, UNIT_RPM);
    status.current = dxl.getPresentCurrent(id_, UNIT_MILLI_AMPERE);
    return status;
  }
};

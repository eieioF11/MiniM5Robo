#include "config/config.hpp"
#include <Arduino.h>
#include <M5Unified.h>

namespace proximity_sensor {
  class LTR553Als {
    uint32_t als_ch0_  = 0;
    uint32_t als_ch1_  = 0;
    uint16_t ps_value_ = 0;
    uint32_t getCh0Value() {
      uint8_t buffer[2];
      uint32_t result;
      M5.In_I2C.readRegister(LTR553_ADDR, 0x8A, buffer, 2, 100000L);
      result = (buffer[1] << 8) | buffer[0];
      return result;
    }

    uint32_t getCh1Value() {
      uint8_t buffer[2];
      uint32_t result;
      M5.In_I2C.readRegister(LTR553_ADDR, 0x88, buffer, 2, 100000L);
      result = (buffer[1] << 8) | buffer[0];
      return result;
    }

    uint16_t getPsValue() {
      uint8_t buffer[2];
      uint16_t result;
      M5.In_I2C.readRegister(LTR553_ADDR, 0x8D, buffer, 2, 100000L);
      buffer[0] &= 0xFF;
      buffer[1] &= 0x07;
      result = (buffer[1] << 8) | buffer[0];
      return result;
    }

  public:
    LTR553Als() {};
    bool begin() {
      // soft reset
      uint8_t value_r = M5.In_I2C.readRegister8(LTR553_ADDR, 0x80, 100000L);
      value_r &= (~0x02);
      uint8_t value_w = value_r | 0x02;
      M5.In_I2C.writeRegister8(LTR553_ADDR, 0x80, value_w, 100000L);

      // PS Led Pluse
      M5.In_I2C.writeRegister8(LTR553_ADDR, 0x83, 0x0F, 100000L);
      // ALS Active Mode
      value_r = M5.In_I2C.readRegister8(LTR553_ADDR, 0x80, 100000L);
      value_r &= (~0x01);
      value_w = value_r | 0x01;
      M5.In_I2C.writeRegister8(LTR553_ADDR, 0x80, value_w, 100000L);
      // PS Active Mode
      value_r = M5.In_I2C.readRegister8(LTR553_ADDR, 0x81, 100000L);
      value_r &= (~0x03);
      value_w = value_r | 0x03;
      M5.In_I2C.writeRegister8(LTR553_ADDR, 0x81, value_w, 100000L);
      return true;
    }
    void update() {
      // Read ALS and PS values
      als_ch0_  = getCh0Value();
      als_ch1_  = getCh1Value();
      ps_value_ = getPsValue();
    }

    std::tuple<uint32_t, uint32_t, uint16_t> getValues() {
      return std::make_tuple(als_ch0_, als_ch1_, ps_value_);
    }
  };
} // namespace proximity_sensor
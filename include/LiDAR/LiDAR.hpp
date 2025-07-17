#pragma once
#include <Arduino.h>
#include <M5Unified.h>
#include <vector>
#include <cmath>

template <typename T>
struct point_t
{
  T x;
  T y;
};

class LiDAR
{
private:
  HardwareSerial LiDAR_serial_;
  typedef enum
  {
    STATE_WAIT_HEADER = 0,
    STATE_READ_HEADER,
    STATE_READ_PAYLOAD,
    STATE_READ_DONE
  } State_t;
  typedef struct
  {
    uint8_t header0;
    uint8_t header1;
    uint8_t header2;
    uint8_t header3;
    uint16_t rotation_speed;
    uint16_t angle_begin;
    uint16_t distance_0;
    uint8_t reserved_0;
    uint16_t distance_1;
    uint8_t reserved_1;
    uint16_t distance_2;
    uint8_t reserved_2;
    uint16_t distance_3;
    uint8_t reserved_3;
    uint16_t distance_4;
    uint8_t reserved_4;
    uint16_t distance_5;
    uint8_t reserved_5;
    uint16_t distance_6;
    uint8_t reserved_6;
    uint16_t distance_7;
    uint8_t reserved_7;
    uint16_t distance_8;
    uint8_t reserved_8;
    uint16_t distance_9;
    uint8_t reserved_9;
    uint16_t distance_10;
    uint8_t reserved_10;
    uint16_t distance_11;
    uint8_t reserved_11;
    uint16_t distance_12;
    uint8_t reserved_12;
    uint16_t distance_13;
    uint8_t reserved_13;
    uint16_t distance_14;
    uint8_t reserved_14;
    uint16_t distance_15;
    uint8_t reserved_15;
    uint16_t angle_end;
    uint16_t crc;
  } __attribute__((packed)) LidarPacket_t;

  const uint8_t header_[4] = {0x55, 0xaa, 0x23, 0x10};

  uint16_t convertDegree(uint16_t input)
  {
    return (input - 40960) / 64;
  }

  uint16_t convertSpeed(uint16_t input)
  {
    return input / 64;
  }

  void remapDegrees(uint16_t minAngle, uint16_t maxAngle, uint16_t *map)
  {
    int16_t delta = maxAngle - minAngle;
    if (maxAngle < minAngle)
    {
      delta += 360;
    }

    if ((map == NULL) || (delta < 0))
    {
      return;
    }
    for (int32_t cnt = 0; cnt < 16; cnt++)
    {
      map[cnt] = minAngle + (delta * cnt / 15);
      if (map[cnt] >= 360)
      {
        map[cnt] -= 360;
      }
    }
  }

  void plot_point(point_t<double> p, uint32_t color)
  {
    if (visualize_)
      M5.Display.drawPixel((uint32_t)(p.x * 100.0) + 160, (uint32_t)(p.y * 100.0) + 120, color);
  }

  void calc_point_cloud(uint16_t *degrees, uint16_t *distances)
  {
    for (int32_t i = 0; i < 16; i++)
    {
      plot_point(point_cloud_[degrees[i]], BLACK); // 描画削除
      point_cloud_[degrees[i]].x = std::cos((1.f * PI * degrees[i]) / 180) * (distances[i] * 0.001);
      point_cloud_[degrees[i]].y = std::sin((1.f * PI * degrees[i]) / 180) * (distances[i] * 0.001);
      if (distances[i] < 1200)
        plot_point(point_cloud_[degrees[i]], WHITE);
    }
  }

  State_t state_;
  uint32_t counter_;
  uint8_t payload_[64];

  std::vector<point_t<double>> point_cloud_; // 360度分の点群

  bool visualize_ = false;

public:
  LiDAR(HardwareSerial &LiDAR_serial) : LiDAR_serial_(LiDAR_serial)
  {
    point_cloud_.resize(360);
  }

  void begin(uint8_t rx, uint8_t tx, bool visualize = true)
  {
    LiDAR_serial_.begin(230400, SERIAL_8N1, rx, tx);
    visualize_ = visualize;
  }

  bool update()
  {
    if (LiDAR_serial_.available())
    {
      uint8_t data = LiDAR_serial_.read();
      switch (state_)
      {
      case STATE_WAIT_HEADER:
        if (data == header_[0])
        {
          counter_++;
          payload_[0] = data;
          state_ = STATE_READ_HEADER;
        }
        else
        {
          // printf("?? (%02X) Please do LiDAR power cycle\n", data);
          LiDAR_serial_.flush();
        }
        break;
      case STATE_READ_HEADER:
        if (data == header_[counter_])
        {
          payload_[counter_] = data;
          counter_++;
          if (counter_ == sizeof(header_))
          {
            state_ = STATE_READ_PAYLOAD;
          }
        }
        else
        {
          counter_ = 0;
          state_ = STATE_WAIT_HEADER;
        }
        break;
      case STATE_READ_PAYLOAD:
        payload_[counter_] = data;
        counter_++;
        if (counter_ == sizeof(LidarPacket_t))
        {
          state_ = STATE_READ_DONE;
        }
        break;
      case STATE_READ_DONE:
        LidarPacket_t *packet = (LidarPacket_t *)payload_;
        {
          uint16_t degree_begin;
          uint16_t degree_end;
          degree_begin = convertDegree(packet->angle_begin);
          degree_end = convertDegree(packet->angle_end);
          if ((degree_begin < 360) && (degree_end < 360))
          {
            printf("%3drpm %5d - %5d\n", convertSpeed(packet->rotation_speed), convertDegree(packet->angle_begin), convertDegree(packet->angle_end));
            uint16_t map[16];
            uint16_t distances[16];
            remapDegrees(degree_begin, degree_end, map);
            distances[0] = packet->distance_0 & 0x3FFF;
            distances[1] = packet->distance_1 & 0x3FFF;
            distances[2] = packet->distance_2 & 0x3FFF;
            distances[3] = packet->distance_3 & 0x3FFF;
            distances[4] = packet->distance_4 & 0x3FFF;
            distances[5] = packet->distance_5 & 0x3FFF;
            distances[6] = packet->distance_6 & 0x3FFF;
            distances[7] = packet->distance_7 & 0x3FFF;
            distances[8] = packet->distance_8 & 0x3FFF;
            distances[9] = packet->distance_9 & 0x3FFF;
            distances[10] = packet->distance_10 & 0x3FFF;
            distances[11] = packet->distance_11 & 0x3FFF;
            distances[12] = packet->distance_12 & 0x3FFF;
            distances[13] = packet->distance_13 & 0x3FFF;
            distances[14] = packet->distance_14 & 0x3FFF;
            distances[15] = packet->distance_15 & 0x3FFF;
            calc_point_cloud(map, distances);
          }
        }
        if (visualize_)
        {
          M5.Display.startWrite();
          M5.Display.setCursor(0, 0);
          M5.Display.printf("Speed : %d rpm  \n", convertSpeed(packet->rotation_speed));
          M5.Display.endWrite();
        }
        state_ = STATE_WAIT_HEADER;
        counter_ = 0;
        state_ = STATE_WAIT_HEADER;
        break;
      }
      return true;
    }
    return false;
  }
  std::vector<point_t<double>> getPointCloud()
  {
    return point_cloud_;
  }
};

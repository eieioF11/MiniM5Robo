#pragma once
#include <Arduino.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>
#include <ratio>
#include <vector>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

namespace common_lib
{

  /**
   * @brief 数学・物理定数
   *
   */
  inline namespace constants {
    constexpr float RPM_TO_MPS = TWO_PI/60.0;
    constexpr float MPS_TO_RPM = 60.0/TWO_PI;
    /// 円周率 / 4
    constexpr float QUARTER_PI = PI / 4.0;

    /// 重力
    constexpr float GRAVITY = 9.807;

    ///
    constexpr float Nm2gfm = (1 / GRAVITY);

    ///
    constexpr float gfm2Nm = GRAVITY;

    ///
    constexpr float mNm2gfcm = (Nm2gfm * 100);

    ///
    constexpr float gfcm2mNm = (gfm2Nm / 100);

    // si unit
    constexpr float m_to_cm = 100.0;
    constexpr float cm_to_m = 0.01;
    constexpr float m_to_mm = 1000.0;
    constexpr float mm_to_m = 0.001;

  } // namespace constants

  // Function
  /**
   * @brief
   *
   * @tparam T
   * @param x
   * @param min
   * @param max
   * @return true
   * @return false
   */
  template <typename T>
  static constexpr bool in_range_open(T x, T min, T max)
  {
    return ((min < x && x < max) ? true : false);
  }

  /**
   * @brief
   *
   * @tparam T
   * @param x
   * @param min
   * @param max
   * @return true
   * @return false
   */
  template <typename T>
  static constexpr bool in_range(T x, T min, T max)
  {
    return ((min <= x && x <= max) ? true : false);
  }

  /**
   * @brief レンジ変換
   *
   * @tparam T
   * @param x
   * @param in_min
   * @param in_max
   * @param out_min
   * @param out_max
   * @return constexpr T
   */
  template <typename T>
  constexpr inline T transform_range(T x, T in_min, T in_max, T out_min, T out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  /**
   * @brief 符号関数
   *
   * @tparam T
   * @param x
   * @return constexpr T
   */
  template <class T>
  constexpr inline T sign(T x)
  {
    return (x > 0) - (x < 0);
  }

  /**
   * @brief 符号関数
   *
   * @tparam T
   * @param x
   * @return constexpr T
   */
  template <class T>
  constexpr inline T sgn(T x)
  {
    return sign<T>(x);
  }

  /**
   * @brief
   *
   * @param angle
   * @return float
   */
  static inline float normalize_angle(float angle)
  {

    float a = std::fmod(angle + PI, TWO_PI) - PI;
    if (a < -PI)
      a += TWO_PI;
    return a;
  }

  /**
   * @brief
   *
   * @param from
   * @param to
   * @return float
   */
  static inline float shortest_angular_distance(float from, float to) { return normalize_angle(to - from); }

  constexpr inline float square(const float x) { return x * x; }
  constexpr inline float cubic(const float x) { return x * x * x; }
  constexpr inline float lerp(const float a, const float b, const float t) { return a + (b - a) * t; }
  constexpr inline bool approx_eq(const float a, const float b, float range = 1e-12) { return (std::abs(a - b) < range); }
  constexpr inline bool approx_zero(const float a, float range = 1e-12) { return (std::abs(a) < range); }

  // Statistics
  /**
   * @brief vector要素の平均値計算
   *
   * @param num_list
   * @return constexpr T
   */
  template <class T>
  constexpr inline T get_average(std::vector<T> num_list)
  {
    T average = std::accumulate(num_list.begin(), num_list.end(), 0.0) / num_list.size();
    return average;
  }

  // Vector
  //  conversion
  /**
   * @brief 二次元ベクトルの型の変換
   *
   * @param BEFORE_VECTOR
   * @return constexpr AFTER_VECTOR
   */
  template <class BEFORE_VECTOR, class AFTER_VECTOR>
  constexpr inline AFTER_VECTOR conversion_vector2(BEFORE_VECTOR in_v)
  {
    AFTER_VECTOR out_v;
    out_v.x = in_v.x;
    out_v.y = in_v.y;
    return out_v;
  }

  /**
   * @brief 三次元ベクトルの型の変換
   *
   * @param BEFORE_VECTOR
   * @return constexpr AFTER_VECTOR
   */
  template <class BEFORE_VECTOR, class AFTER_VECTOR>
  constexpr inline AFTER_VECTOR conversion_vector3(BEFORE_VECTOR in_v)
  {
    AFTER_VECTOR out_v;
    out_v.x = in_v.x;
    out_v.y = in_v.y;
    out_v.z = in_v.z;
    return out_v;
  }

  // length
  /**
   * @brief 二次元ベクトルの長さ
   *
   * @param v
   * @return constexpr float
   */
  template <class POINT>
  constexpr inline float length(const POINT &v)
  {
    return std::hypot(v.x, v.y);
  }

  /**
   * @brief 三次元ベクトルの長さ
   *
   * @param v
   * @return constexpr float
   */
  template <class POINT>
  constexpr inline float length_3d(const POINT &v)
  {
    return std::hypot(v.x, v.y, v.z);
  }

  // distance
  /**
   * @brief 二点距離
   *
   * @param a
   * @param b
   * @return constexpr float
   */
  template <class POINT>
  constexpr inline float distance(const POINT &a, const POINT &b)
  {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  /**
   * @brief 二点距離
   *
   * @param a
   * @param b
   * @return constexpr float
   */
  template <class POINT>
  constexpr inline float distance_3d(const POINT &a, const POINT &b)
  {
    return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
  }

  // dot
  /**
   * @brief 内積
   *
   * @param a
   * @param b
   * @return constexpr float
   */
  template <class POINT>
  constexpr inline float dot(const POINT &a, const POINT &b)
  {
    return a.x * b.x + a.y * b.y;
  }

  /**
   * @brief 内積
   *
   * @param a
   * @param b
   * @return constexpr float
   */
  template <class POINT>
  constexpr inline float dot_3d(const POINT &a, const POINT &b)
  {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  }

  // rotation
  /**
   * @brief ２次元の点回転
   *
   * @param in
   * @param theta
   * @return constexpr POINT
   */
  template <class POINT>
  constexpr inline POINT rotation(const POINT &in, float theta)
  {
    POINT out;
    out.x = in.x * std::cos(theta) - in.y * std::sin(theta);
    out.y = in.x * std::sin(theta) + in.y * std::cos(theta);
    return out;
  }

} // namespace common_lib

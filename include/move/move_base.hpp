#pragma once
#include "utility/math_util.hpp"
#include <cmath>
#include <iostream>
#include <numeric>
#include <ratio>
#include <vector>

namespace kinematics {

  /**
   * @brief 逆運動学モデルの基底クラス
   *
   */
  template <typename FLOATING_TYPE>
  class MoveBase;
  using MoveBasef = MoveBase<float>;
  using MoveBased = MoveBase<double>;

  template <typename FLOATING_TYPE>
  class MoveBase {
    using value_type = FLOATING_TYPE;

  public:
    MoveBase() {}
    MoveBase(value_type r, value_type l, value_type w, value_type h) : R(r), L(l), WIDTH(w), HEIGHT(h) {}

    /**
     * @brief 3次元の速度入力
     *
     * @param vx [m/s]
     * @param vy [m/s]
     * @param vz [m/s]
     * @param angular_x [rad/s]
     * @param angular_y [rad/s]
     * @param angular_z [rad/s]
     */
    virtual void move(value_type vx, value_type vy, value_type vz, value_type angular_x, value_type angular_y, value_type angular_z) {}
    /**
     * @brief 2次元の速度入力
     *
     * @param vx [m/s]
     * @param vy [m/s]
     * @param angular_z [rad/s]
     */
    void move(value_type vx, value_type vy, value_type angular_z) { move(vx, vy, 0., 0., 0., angular_z); }
    /**
     * @brief 停止動作
     *
     */
    virtual void stop() { move(0., 0., 0., 0., 0., 0.); }

    /**
     * @brief 車輪の速度を取得
     *
     * @return std::vector<value_type>
     */
    std::vector<value_type> get_wheel_speeds() const { return wheel_speeds_; }

  protected:
    std::vector<value_type> wheel_speeds_; // [rps]
    // robot parameter
    value_type R; // wheel radius [m]
    value_type L; // half intervals between wheels [m]
    value_type WIDTH;
    value_type HEIGHT;
  };
} // namespace kinematics

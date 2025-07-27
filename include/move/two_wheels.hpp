#pragma once
#include <cmath>
#include <iostream>
#include <numeric>
#include <ratio>
#include <vector>

#include "move_base.hpp"

namespace kinematics {

  /**
   * @brief 対向二輪型ロボット逆運動学モデル
   *
   */
  template <typename FLOATING_TYPE>
  class TwoWheels;
  using TwoWheelsf = TwoWheels<float>;
  using TwoWheelsd = TwoWheels<double>;

  template <typename FLOATING_TYPE>
  class TwoWheels : public MoveBase<FLOATING_TYPE> {
    using value_type = FLOATING_TYPE;

  private:
    value_type inv_r_;

  public:
    TwoWheels() : MoveBase<value_type>() {}
    TwoWheels(value_type r, value_type l) : MoveBase<value_type>(r, l, 0., 0.) { // r:wheel radius[m], l:diameter[m]
      inv_r_ = 1. / r;
      this->wheel_speeds_.resize(2);
    }

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
    void move(value_type vx, value_type vy, value_type vz, value_type angular_x, value_type angular_y, value_type angular_z) {
      this->wheel_speeds_[0] = (vx * inv_r_ - (angular_z * this->L * inv_r_)) * common_utils::constants::RADPS_TO_RPS;
      this->wheel_speeds_[1] = (vx * inv_r_ + (angular_z * this->L * inv_r_)) * common_utils::constants::RADPS_TO_RPS;
    }
  };
  /*---------------------
  left wheel(l):0
  right wheel(r):1
    |-L-|
  l[0]--|--[1]r
   x
   |
  _|____y
  o|
  ----------------------*/
} // namespace kinematics

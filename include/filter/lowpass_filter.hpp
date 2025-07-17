#pragma once
// std
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>

namespace common_lib {

  template <typename FLOATING_TYPE>
  class LowpassFilter;
  using LowpassFilterf = LowpassFilter<float>;
  using LowpassFilterd = LowpassFilter<double>;

  template <typename FLOATING_TYPE>
  class LowpassFilter {
    using value_type = FLOATING_TYPE;

  public:
    /**
     * @brief Construct a new Lowpass Filter object
     *
     * @param alpha : 0.0~1.0の値
     */
    LowpassFilter(value_type alpha) : alpha_(alpha), pre_val_(0.0) {};
    /**
     * @brief alphaの設定
     *
     * @param alpha : 0.0~1.0の値
     */
    void set_alpha(value_type alpha) { alpha_ = alpha; }
    /**
     * @brief filtering
     *
     * @param val : 高周波ノイズの値(ex.加速度センサの値)
     */
    value_type filtering(value_type val) {
      value_type lpf = (1.0 - alpha_) * pre_val_ + alpha_ * val;
      pre_val_ = lpf;
      return lpf;
    }

  private:
    value_type alpha_;
    value_type pre_val_;
  };
} // namespace common_lib
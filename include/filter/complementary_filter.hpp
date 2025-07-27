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
  class ComplementaryFilter;
  using ComplementaryFilterf = ComplementaryFilter<float>;
  using ComplementaryFilterd = ComplementaryFilter<double>;

  template <typename FLOATING_TYPE>
  class ComplementaryFilter {
    using value_type = FLOATING_TYPE;

  public:
    /**
     * @brief Construct a new Complementary Filter object
     *
     * @param alpha : 0.0~1.0の値
     */
    ComplementaryFilter(value_type alpha) : alpha_(alpha){};
    /**
     * @brief alphaの設定
     *
     * @param alpha : 0.0~1.0の値
     */
    void set_alpha(value_type alpha) { alpha_ = alpha; }
    /**
     * @brief filtering
     *
     * @param h_val : 高周波ノイズの値(ex.加速度センサの値)
     * @param l_val : 低周波ノイズの値(ex.ジャイロセンサの値)
     */
    value_type filtering(value_type h_val, value_type l_val) { return alpha_ * l_val + (1 - alpha_) * h_val; }

  private:
    value_type alpha_;
  };
} // namespace common_lib
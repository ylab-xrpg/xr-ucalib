// Copyright 2026 Yongjiang Laboratory
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <ceres/ceres.h>
#include <ceres/dynamic_autodiff_cost_function.h>

#include "xr_ucalib/uc_unified_calib/spline/ceres_spline_helper_jet.hpp"
#include "xr_ucalib/uc_unified_calib/spline/spline_meta.hpp"

namespace xr_ucalib {

// Structure for magnetometer measurement cost function.
template <int Order>
struct MagCost {
 public:
  /**
   * @brief Construct a magnetometer cost function object.
   *
   * @param[in] rot_meta Meta data for the rotation spline.
   * @param[in] mag_frame Magnetometer frame with measurement information.
   * @param[in] weight Residual weight.
   */
  MagCost(const SplineMeta<Order>& rot_meta, const MagFrame::Ptr& mag_frame,
          const double& weight)
      : rot_meta_(rot_meta),
        mag_frame_(mag_frame),
        weight_(weight),
        rot_dt_inv_(1. / rot_meta.segments.front().knot_interval) {}

  // Create an instance of magnetometer cost function wrapped in a dynamic
  // auto-diff cost function.
  static auto Create(const SplineMeta<Order>& rot_meta,
                     const MagFrame::Ptr& mag_frame, const double& weight) {
    return new ceres::DynamicAutoDiffCostFunction<MagCost>(
        new MagCost(rot_meta, mag_frame, weight));
  }

  /**
   * @brief Overload operator, define the residual calculation method and bind
   * parameter blocks.
   *
   * The parameters blocks include:
   * [ rot_G_B_knot   | ... | rot_G_B_knot   |
   *   rot_B_Mi | mag_in_W | toff_B_Mi ]
   *
   * @tparam T Type of the parameter. (compatible with ceres::Jet for automatic
   * differentiation)
   * @param params Pointer to the parameter blocks.
   * @param residuals Pointer to the output residuals.
   * @return true If the calculation is completed.
   */
  template <class T>
  bool operator()(T const* const* params, T* residuals) const {
    // Determine the address offset.
    size_t ROT_KNOT_OFFSET = 0;
    size_t ROT_B_Mi_OFFSET = rot_meta_.NumParameters();
    size_t MAG_IN_W_OFFSET = ROT_B_Mi_OFFSET + 1;
    size_t TOFF_B_Mi_OFFSET = MAG_IN_W_OFFSET + 1;

    // Get the values for time-invariant parameters.
    Eigen::Map<const Sophus::SO3<T>> rot_B_Mi(params[ROT_B_Mi_OFFSET]);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> mag_in_W(params[MAG_IN_W_OFFSET]);
    T toff_B_Mi = params[TOFF_B_Mi_OFFSET][0];

    // Determine the address offset for spline knots.
    T t_M = toff_B_Mi + T(mag_frame_->timestamp);
    // For measurements outside the B-spline time range, we simply set their
    // gradients to zero.
    size_t rot_spline_index;
    T rot_spline_fraction;
    if (!rot_meta_.ComputeSplineIndex(t_M, rot_spline_index,
                                      rot_spline_fraction)) {
      Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);
      res.setZero();
      return true;
    }

    ROT_KNOT_OFFSET = rot_spline_index;

    // Evaluate rotation from the spline.
    Sophus::SO3<T> rot_W_B;
    CeresSplineHelperJet<T, Order>::template EvaluateLie<Sophus::SO3>(
        params + ROT_KNOT_OFFSET, rot_spline_fraction, rot_dt_inv_, &rot_W_B);

    // Compute the predicted magnetometer measurement in the body frame.
    Eigen::Matrix<T, 3, 1> mag_in_M_pred =
        rot_B_Mi.inverse() * (rot_W_B.inverse() * mag_in_W);

    // Get the magnetometer measurement.
    Eigen::Matrix<T, 3, 1> mag_meas = mag_frame_->mag.cast<T>();

    // Compute residuals.
    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);
    res = T(weight_) * (mag_in_M_pred - mag_meas);

    return true;
  }

 private:
  // Spline meta data.
  SplineMeta<Order> rot_meta_;

  // Magnetometer frame with measurement information.
  MagFrame::Ptr mag_frame_;

  // Residual weight.
  double weight_;

  // Inverse of the time interval for rotation spline.
  double rot_dt_inv_;
};

}  // namespace xr_ucalib
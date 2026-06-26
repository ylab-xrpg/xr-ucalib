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

#include "xr_ucalib/uc_common/sensor_data/sensor_data_types.h"
#include "xr_ucalib/uc_unified_calib/spline/ceres_spline_helper_jet.hpp"
#include "xr_ucalib/uc_unified_calib/spline/spline_meta.hpp"

namespace xr_ucalib {

// Structure for gyroscope measurement cost function.
template <int Order>
struct ImuGyrCost {
 public:
  /**
   * @brief Construct a IMU gyroscope cost function object.
   *
   * @param[in] rot_meta Meta data for the rotation spline.
   * @param[in] imu_frame IMU frame with measurement information.
   * @param[in] weight Residual weight.
   */
  ImuGyrCost(const SplineMeta<Order> &rot_meta, const ImuFrame::Ptr &imu_frame,
             const double &weight)
      : rot_meta_(rot_meta),
        imu_frame_(imu_frame),
        weight_(weight),
        rot_dt_inv_(1. / rot_meta.segments.front().knot_interval) {}

  // Create an instance of gyroscope cost function wrapped in a dynamic
  // auto-diff cost function.
  static auto Create(const SplineMeta<Order> &rot_meta,
                     const ImuFrame::Ptr &imu_frame, const double &weight) {
    return new ceres::DynamicAutoDiffCostFunction<ImuGyrCost>(
        new ImuGyrCost(rot_meta, imu_frame, weight));
  }

  /**
   * @brief Overload operator, define the residual calculation method and bind
   * parameter blocks.
   *
   * The parameters blocks include:
   * [ rot_W_B_knot   | ... | rot_W_B_knot   |
   *   rot_B_I | toff_B_I |
   *   gyr_bias | gyr_scale | gyr_non_ortho | rot_gyr_acc ]
   *
   * @tparam T Type of the parameter. (compatible with ceres::Jet for automatic
   * differentiation)
   * @param params Pointer to the parameter blocks.
   * @param residuals Pointer to the output residuals.
   * @return true If the calculation is completed.
   */
  template <class T>
  bool operator()(T const *const *params, T *residuals) const {
    // Determine the address offsets.
    size_t ROT_KNOT_OFFSET;
    size_t ROT_B_I_OFFSET = rot_meta_.NumParameters();
    size_t TOFF_B_I_OFFSET = ROT_B_I_OFFSET + 1;
    size_t GYR_BIAS_OFFSET = TOFF_B_I_OFFSET + 1;
    size_t GYR_SCALE_OFFSET = GYR_BIAS_OFFSET + 1;
    size_t GYR_NON_ORTHO_OFFSET = GYR_SCALE_OFFSET + 1;
    size_t ROT_GYR_ACC_OFFSET = GYR_NON_ORTHO_OFFSET + 1;

    // Get the values for time-invariant parameters.
    Eigen::Map<Sophus::SO3<T> const> const rot_B_I(params[ROT_B_I_OFFSET]);
    T toff_B_I = params[TOFF_B_I_OFFSET][0];
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> gyr_bias(params[GYR_BIAS_OFFSET]);

    Eigen::Matrix<T, 3, 3> gyr_map_mat = Eigen::Matrix<T, 3, 3>::Zero();
    gyr_map_mat.diagonal() =
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(params[GYR_SCALE_OFFSET]);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> gyr_non_ortho(
        params[GYR_NON_ORTHO_OFFSET]);
    gyr_map_mat(0, 1) = gyr_non_ortho(0);
    gyr_map_mat(0, 2) = gyr_non_ortho(1);
    gyr_map_mat(1, 2) = gyr_non_ortho(2);

    Eigen::Map<Sophus::SO3<T> const> const rot_gyr_acc(
        params[ROT_GYR_ACC_OFFSET]);

    // Determine the address offset for spline knots.
    T t_I = toff_B_I + T(imu_frame_->timestamp);
    // For measurements outside the B-spline time range, we simply set their
    // gradients to zero.
    size_t rot_spline_index;
    T rot_spline_fraction;
    if (!rot_meta_.ComputeSplineIndex(t_I, rot_spline_index,
                                      rot_spline_fraction)) {
      Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);
      res.setZero();
      return true;
    }

    ROT_KNOT_OFFSET = rot_spline_index;

    // Evaluate rotation and angular velocity from the spline.
    Sophus::SO3<T> rot_W_B;
    typename Sophus::SO3<T>::Tangent rot_vel_B_B;
    CeresSplineHelperJet<T, Order>::template EvaluateLie<Sophus::SO3>(
        params + ROT_KNOT_OFFSET, rot_spline_fraction, rot_dt_inv_, &rot_W_B,
        &rot_vel_B_B);

    // Calculate the predicted values.
    Eigen::Matrix<T, 3, 1> gyr_pred =
        gyr_map_mat * (rot_gyr_acc * rot_B_I.inverse() * rot_vel_B_B) +
        gyr_bias;

    // Get the measurements.
    Eigen::Matrix<T, 3, 1> gyr_meas = imu_frame_->gyr.template cast<T>();

    // Calculates the residuals based on the difference between the measurements
    // and predicted values.
    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);
    res = T(weight_) * (gyr_pred - gyr_meas);

    return true;
  }

 private:
  // Spline meta data.
  SplineMeta<Order> rot_meta_;

  // IMU frame with measurement information.
  ImuFrame::Ptr imu_frame_;

  // Residual weights.
  double weight_;

  // Inverses of the time intervals for splines.
  double rot_dt_inv_;
};

}  // namespace xr_ucalib

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

// Structure for IMU accelerometer cost function.
template <int Order>
struct ImuAccCost {
 public:
  /**
   * @brief Construct an IMU accelerometer cost function object.
   *
   * @param[in] trans_meta Meta data for the translation spline.
   * @param[in] rot_meta Meta data for the rotation spline.
   * @param[in] imu_frame IMU frame with measurement information.
   * @param[in] weight Residual weight.
   */
  ImuAccCost(const SplineMeta<Order> &trans_meta,
             const SplineMeta<Order> &rot_meta, const ImuFrame::Ptr imu_frame,
             const double &weight)
      : trans_meta_(trans_meta),
        rot_meta_(rot_meta),
        imu_frame_(imu_frame),
        weight_(weight),
        trans_dt_inv_(1. / trans_meta.segments.front().knot_interval),
        rot_dt_inv_(1. / rot_meta.segments.front().knot_interval) {}

  // Create an instance of accelerometer cost function wrapped in a dynamic
  // auto-diff cost function.
  static auto Create(const SplineMeta<Order> &trans_meta,
                     const SplineMeta<Order> &rot_meta,
                     const ImuFrame::Ptr imu_frame, const double &weight) {
    return new ceres::DynamicAutoDiffCostFunction<ImuAccCost>(
        new ImuAccCost(trans_meta, rot_meta, imu_frame, weight));
  }

  /**
   * @brief Overload operator, define the residual calculation method and bind
   * parameter blocks.
   *
   * The parameters blocks include:
   * [ trans_W_B_knot | ... | trans_W_B_knot |
   *   rot_W_B_knot   | ... | rot_W_B_knot   |
   *   trans_B_I | rot_B_I | gravity_in_W | toff_B_I |
   *   acc_bias  | acc_scale | acc_non_ortho ]
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
    size_t TRANS_KNOT_OFFSET;
    size_t ROT_KNOT_OFFSET;
    size_t TRANS_B_I_OFFSET =
        trans_meta_.NumParameters() + rot_meta_.NumParameters();
    size_t ROT_B_I_OFFSET = TRANS_B_I_OFFSET + 1;
    size_t GRAVITY_W_OFFSET = ROT_B_I_OFFSET + 1;
    size_t TOFF_B_I_OFFSET = GRAVITY_W_OFFSET + 1;
    size_t ACC_BIAS_OFFSET = TOFF_B_I_OFFSET + 1;
    size_t ACC_SCALE_OFFSET = ACC_BIAS_OFFSET + 1;
    size_t ACC_NON_ORTHO_OFFSET = ACC_SCALE_OFFSET + 1;

    // Get the values for time-invariant parameters.
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> trans_B_I(
        params[TRANS_B_I_OFFSET]);
    Eigen::Map<const Sophus::SO3<T>> rot_B_I(params[ROT_B_I_OFFSET]);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> gravity_in_W(
        params[GRAVITY_W_OFFSET]);
    T toff_B_I = params[TOFF_B_I_OFFSET][0];
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> acc_bias(params[ACC_BIAS_OFFSET]);

    Eigen::Matrix<T, 3, 3> acc_map_mat = Eigen::Matrix<T, 3, 3>::Zero();
    acc_map_mat.diagonal() =
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(params[ACC_SCALE_OFFSET]);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> acc_non_ortho(
        params[ACC_NON_ORTHO_OFFSET]);
    acc_map_mat(0, 1) = acc_non_ortho(0);
    acc_map_mat(0, 2) = acc_non_ortho(1);
    acc_map_mat(1, 2) = acc_non_ortho(2);

    // Determine the address offset for spline knots.
    T t_I = toff_B_I + T(imu_frame_->timestamp);
    // For measurements outside the B-spline time range, we simply set their
    // gradients to zero.
    size_t trans_spline_index, rot_spline_index;
    T trans_spline_fraction, rot_spline_fraction;
    if (!trans_meta_.ComputeSplineIndex(t_I, trans_spline_index,
                                        trans_spline_fraction) ||
        !rot_meta_.ComputeSplineIndex(t_I, rot_spline_index,
                                      rot_spline_fraction)) {
      Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);
      res.setZero();
      return true;
    }

    TRANS_KNOT_OFFSET = trans_spline_index;
    ROT_KNOT_OFFSET = trans_meta_.NumParameters() + rot_spline_index;

    // Evaluate pose and inertial information from the spline.
    Eigen::Matrix<T, 3, 1> trans_acc_W_B;
    CeresSplineHelperJet<T, Order>::template Evaluate<3, 2>(
        params + TRANS_KNOT_OFFSET, trans_spline_fraction, trans_dt_inv_,
        &trans_acc_W_B);

    Sophus::SO3<T> rot_W_B;
    typename Sophus::SO3<T>::Tangent rot_vel_B_B, rot_acc_B_B;
    CeresSplineHelperJet<T, Order>::template EvaluateLie<Sophus::SO3>(
        params + ROT_KNOT_OFFSET, rot_spline_fraction, rot_dt_inv_, &rot_W_B,
        &rot_vel_B_B, &rot_acc_B_B);
    // typename Sophus::SO3<T>::Tangent rot_vel_W_B = rot_W_B * rot_vel_B_B;
    // typename Sophus::SO3<T>::Tangent rot_acc_W_B = rot_W_B * rot_acc_B_B;

    // Calculate the intermediate variable.
    Eigen::Matrix<T, 3, 3> rot_vel_B_B_hat = Sophus::SO3<T>::hat(rot_vel_B_B);
    Eigen::Matrix<T, 3, 3> rot_acc_B_B_hat = Sophus::SO3<T>::hat(rot_acc_B_B);
    // Eigen::Matrix<T, 3, 3> rot_vel_B_B_hat =
    // Sophus::SO3<T>::hat(rot_vel_W_B); Eigen::Matrix<T, 3, 3> rot_acc_B_B_hat
    // = Sophus::SO3<T>::hat(rot_acc_W_B);
    Eigen::Matrix<T, 3, 1> trans_acc_W_I =
        trans_acc_W_B +
        rot_W_B.matrix() *
            (rot_acc_B_B_hat + rot_vel_B_B_hat * rot_vel_B_B_hat) * trans_B_I;
    // Eigen::Matrix<T, 3, 1> trans_acc_W_I =
    //     trans_acc_W_B + (rot_acc_B_B_hat + rot_vel_B_B_hat * rot_vel_B_B_hat)
    //     * (rot_W_B.matrix() * trans_B_I);

    // Calculate the predicted values.
    Eigen::Matrix<T, 3, 1> acc_pred =
        acc_map_mat *
            ((rot_W_B * rot_B_I).inverse() * (trans_acc_W_I + gravity_in_W)) +
        acc_bias;

    // Get the measurements.
    Eigen::Matrix<T, 3, 1> acc_meas = imu_frame_->acc.template cast<T>();

    // Calculates the residuals based on the difference between the measurements
    // and predicted values.
    Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);
    res = T(weight_) * (acc_pred - acc_meas);

    return true;
  }

 private:
  // Spline meta data.
  SplineMeta<Order> trans_meta_, rot_meta_;

  // IMU frame with measurement information.
  ImuFrame::Ptr imu_frame_;

  // Residual weights.
  double weight_;

  // Inverses of the time intervals for splines.
  double trans_dt_inv_, rot_dt_inv_;
};

}  // namespace xr_ucalib

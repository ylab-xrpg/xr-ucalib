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

#include "xr_ucalib/uc_common/calib_parameter/cam_eqdist_intrinsic.hpp"
#include "xr_ucalib/uc_common/calib_parameter/cam_radtan_intrinsic.hpp"
#include "xr_ucalib/uc_common/sensor_data/sensor_data_types.h"
#include "xr_ucalib/uc_unified_calib/spline/ceres_spline_helper_jet.hpp"
#include "xr_ucalib/uc_unified_calib/spline/spline_meta.hpp"

namespace xr_ucalib {

// Structure for camera reprojection cost function in continuous-time setting.
template <int Order>
struct CamReprojCtCost {
 public:
  /**
   * @brief Construct a camera reprojection cost function object.
   *
   * @param[in] trans_meta Meta data for the translation spline.
   * @param[in] rot_meta Meta data for the rotation spline.
   * @param[in] cam_intrinsic Camera intrinsic pointer.
   * @param[in] target_point3d 3D point in the target local frame.
   * @param[in] image_point2d 2D image point.
   * @param[in] timestamp Timestamp of the image point.
   * @param[in] weight Residual weight.
   */
  CamReprojCtCost(const SplineMeta<Order> &trans_meta,
                  const SplineMeta<Order> &rot_meta,
                  const CamIntrinsicBase::Ptr cam_intrinsic,
                  const Eigen::Vector3d &target_point3d,
                  const Eigen::Vector2d &image_point2d, const double &timestamp,
                  const double &weight)
      : trans_meta_(trans_meta),
        rot_meta_(rot_meta),
        cam_intrinsic_(cam_intrinsic),
        target_point3d_(target_point3d),
        image_point2d_(image_point2d),
        timestamp_(timestamp),
        weight_(weight),
        trans_dt_inv_(1. / trans_meta.segments.front().knot_interval),
        rot_dt_inv_(1. / rot_meta.segments.front().knot_interval) {}

  // Create an instance of camera reprojection cost function wrapped in a
  // dynamic auto-diff cost function.
  static auto Create(const SplineMeta<Order> &trans_meta,
                     const SplineMeta<Order> &rot_meta,
                     const CamIntrinsicBase::Ptr cam_intrinsic,
                     const Eigen::Vector3d &target_point3d,
                     const Eigen::Vector2d &image_point2d,
                     const double &timestamp, const double &weight) {
    return new ceres::DynamicAutoDiffCostFunction<CamReprojCtCost>(
        new CamReprojCtCost(trans_meta, rot_meta, cam_intrinsic, target_point3d,
                            image_point2d, timestamp, weight));
  }

  /**
   * @brief Overload operator, define the residual calculation method and bind
   * parameter blocks.
   *
   * The parameters blocks include:
   * [ trans_W_B_knot | ... | trans_W_B_knot |
   *   rot_W_B_knot   | ... | rot_W_B_knot   |
   *   trans_B_Cb | rot_B_Cb | trans_Cb_Ci | rot_Cb_Ci |
   *   trans_W_Ti | rot_W_Ti | toff_B_Cb | toff_Cb_Ci |
   *   cam_intrinsics ]
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
    size_t TRANS_B_Cb_OFFSET =
        trans_meta_.NumParameters() + rot_meta_.NumParameters();
    size_t ROT_B_Cb_OFFSET = TRANS_B_Cb_OFFSET + 1;
    size_t TRANS_Cb_Ci_OFFSET = ROT_B_Cb_OFFSET + 1;
    size_t ROT_Cb_Ci_OFFSET = TRANS_Cb_Ci_OFFSET + 1;
    size_t TRANS_W_Ti_OFFSET = ROT_Cb_Ci_OFFSET + 1;
    size_t ROT_W_Ti_OFFSET = TRANS_W_Ti_OFFSET + 1;
    size_t TOFF_B_Cb_OFFSET = ROT_W_Ti_OFFSET + 1;
    size_t TOFF_Cb_Ci_OFFSET = TOFF_B_Cb_OFFSET + 1;
    size_t CAM_INTRINSIC_OFFSET = TOFF_Cb_Ci_OFFSET + 1;

    // Get the values for time-invariant parameters.
    Eigen::Matrix<T, 3, 1> trans_B_Cb =
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(params[TRANS_B_Cb_OFFSET]);
    Sophus::SO3<T> rot_B_Cb =
        Eigen::Map<const Sophus::SO3<T>>(params[ROT_B_Cb_OFFSET]);
    Eigen::Matrix<T, 3, 1> trans_Cb_Ci =
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(params[TRANS_Cb_Ci_OFFSET]);
    Sophus::SO3<T> rot_Cb_Ci =
        Eigen::Map<const Sophus::SO3<T>>(params[ROT_Cb_Ci_OFFSET]);
    Eigen::Matrix<T, 3, 1> trans_W_Ti =
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(params[TRANS_W_Ti_OFFSET]);
    Sophus::SO3<T> rot_W_Ti =
        Eigen::Map<const Sophus::SO3<T>>(params[ROT_W_Ti_OFFSET]);
    T toff_B_Cb = params[TOFF_B_Cb_OFFSET][0];
    T toff_Cb_Ci = params[TOFF_Cb_Ci_OFFSET][0];
    // TODO: The dimension of camera intrinsic parameters may vary for different
    // camera models.
    const Eigen::Matrix<T, 8, 1> cam_intrinsics =
        Eigen::Map<const Eigen::Matrix<T, 8, 1>>(params[CAM_INTRINSIC_OFFSET]);

    // Determine the address offset for spline knots.
    T t_Ci = toff_B_Cb + toff_Cb_Ci + T(timestamp_);
    // For measurements outside the B-spline time range, we simply set their
    // gradients to zero.
    size_t trans_spline_index, rot_spline_index;
    T trans_spline_fraction, rot_spline_fraction;
    if (!trans_meta_.ComputeSplineIndex(t_Ci, trans_spline_index,
                                        trans_spline_fraction) ||
        !rot_meta_.ComputeSplineIndex(t_Ci, rot_spline_index,
                                      rot_spline_fraction)) {
      Eigen::Map<Eigen::Matrix<T, 2, 1>> res(residuals);
      res.setZero();
      return true;
    }

    TRANS_KNOT_OFFSET = trans_spline_index;
    ROT_KNOT_OFFSET = trans_meta_.NumParameters() + rot_spline_index;

    // Evaluate pose from the spline.
    Eigen::Matrix<T, 3, 1> trans_W_B;
    CeresSplineHelperJet<T, Order>::template Evaluate<3, 0>(
        params + TRANS_KNOT_OFFSET, trans_spline_fraction, trans_dt_inv_,
        &trans_W_B);

    Sophus::SO3<T> rot_W_B;
    CeresSplineHelperJet<T, Order>::template EvaluateLie<Sophus::SO3>(
        params + ROT_KNOT_OFFSET, rot_spline_fraction, rot_dt_inv_, &rot_W_B);

    // Compute i-th camera pose: T_W_Ci = T_W_B * T_B_Cb * T_Cb_Ci.
    Eigen::Matrix<T, 3, 1> trans_W_Ci =
        rot_W_B * (rot_B_Cb * trans_Cb_Ci + trans_B_Cb) + trans_W_B;
    Sophus::SO3<T> rot_W_Ci = rot_W_B * rot_B_Cb * rot_Cb_Ci;

    // Transform the 3D point from target frame to world frame.
    Eigen::Matrix<T, 3, 1> target_point_3d_in_Ti = target_point3d_.cast<T>();
    Eigen::Matrix<T, 3, 1> target_point_3d_in_W =
        rot_W_Ti * target_point_3d_in_Ti + trans_W_Ti;

    // Project the 3D point to 2D image plane.
    Eigen::Matrix<T, 2, 1> projected_point_2d;
    // Now we only support radial-tangential and equidistant camera models.
    if (auto ptr =
            std::dynamic_pointer_cast<CamRadtanIntrinsic>(cam_intrinsic_)) {
      CamRadtanIntrinsic::Space2Image(
          trans_W_Ci, rot_W_Ci.unit_quaternion(), target_point_3d_in_W,
          cam_intrinsics.data(), projected_point_2d);
    } else if (auto ptr = std::dynamic_pointer_cast<CamEqdistIntrinsic>(
                   cam_intrinsic_)) {
      CamEqdistIntrinsic::Space2Image(
          trans_W_Ci, rot_W_Ci.unit_quaternion(), target_point_3d_in_W,
          cam_intrinsics.data(), projected_point_2d);
    } else {
      // Unsupported camera model.
      return false;
    }

    // Compute residuals (reprojection error).
    Eigen::Map<Eigen::Matrix<T, 2, 1>> res(residuals);
    Eigen::Matrix<T, 2, 1> observed_point_2d = image_point2d_.cast<T>();
    res = T(weight_) * (projected_point_2d - observed_point_2d);

    return true;
  }

 private:
  // Spline meta data.
  SplineMeta<Order> trans_meta_, rot_meta_;

  // Camera intrinsic that used for determining the projection model.
  CamIntrinsicBase::Ptr cam_intrinsic_;

  // Fiducial 3D corner in target local frame.
  Eigen::Vector3d target_point3d_;

  // 2D image point.
  Eigen::Vector2d image_point2d_;

  // Timestamp of the image point.
  double timestamp_;

  // Residual weights.
  double weight_;

  // Inverses of the time intervals for splines.
  double trans_dt_inv_, rot_dt_inv_;
};

}  // namespace xr_ucalib
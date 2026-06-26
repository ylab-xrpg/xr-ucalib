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

// clang-format off
#include <memory>

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <Eigen/Eigen>

#include "xr_ucalib/uc_common/calib_parameter/cam_eqdist_intrinsic.hpp"
#include "xr_ucalib/uc_common/calib_parameter/cam_radtan_intrinsic.hpp"
// clang-format on

namespace xr_ucalib {

class CamReprojCost {
 public:
  CamReprojCost(const CamIntrinsicBase::Ptr& cam_intrinsic,
                const Eigen::Vector3d& target_point3d,
                const Eigen::Vector2d& iamge_point2d,
                const double& weight = 1.0)
      : cam_intrinsic_(cam_intrinsic),
        target_point3d_(target_point3d),
        iamge_point2d_(iamge_point2d),
        weight_(weight) {}

  // Factory method to create a Ceres cost function instance.
  // TODO: The camera model in our system currently only supports 'radtan' and
  // 'equidistant', which have 8 intrinsic parameters. So we hardcode the size
  // of the intrinsic parameter block to 8 here. To support more camera models,
  // we may need to generalize this part.
  static ceres::CostFunction* Create(const CamIntrinsicBase::Ptr& cam_intrinsic,
                                     const Eigen::Vector3d& target_point3d,
                                     const Eigen::Vector2d& iamge_point2d,
                                     const double& weight = 1.0) {
    return new ceres::AutoDiffCostFunction<CamReprojCost, 2, 3, 4, 3, 4, 8, 3,
                                           4>(new CamReprojCost(
        cam_intrinsic, target_point3d, iamge_point2d, weight));
  }

  /**
   * @brief Ceres operator() to compute reprojection residuals.
   *
   * @tparam T Template type for Ceres automatic differentiation.
   * @param trans_W_Cb_ptr Translation of base camera in world frame.
   * @param rot_W_Cb_ptr  Rotation from base camera to world (as quaternion).
   * @param trans_Cb_Ci_ptr Translation of i-th camera in base camera frame.
   * @param rot_Cb_Ci_ptr Rotation from i-th camera to base camera (as
   * quaternion).
   * @param cam_intrinsics_ptr Camera intrinsic parameters.
   * @param trans_W_Ti_ptr Translation of target i in world frame.
   * @param rot_W_Ti_ptr Rotation from target i to world (as quaternion).
   * @param residuals_ptr Output residuals (2D reprojection error).
   * @return true If successful, false otherwise.
   */
  template <class T>
  bool operator()(const T* const trans_W_Cb_ptr, const T* const rot_W_Cb_ptr,
                  const T* const trans_Cb_Ci_ptr, const T* const rot_Cb_Ci_ptr,
                  const T* const cam_intrinsics_ptr,
                  const T* const trans_W_Ti_ptr, const T* const rot_W_Ti_ptr,
                  T* residuals_ptr) const {
    // Map input parameters to Eigen types.
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> trans_W_Cb(trans_W_Cb_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> rot_W_Cb(rot_W_Cb_ptr);

    Eigen::Map<const Eigen::Matrix<T, 3, 1>> trans_Cb_Ci(trans_Cb_Ci_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> rot_Cb_Ci(rot_Cb_Ci_ptr);

    Eigen::Map<const Eigen::Matrix<T, 3, 1>> trans_W_Ti(trans_W_Ti_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> rot_W_Ti(rot_W_Ti_ptr);

    // Compute i-th camera pose.
    Eigen::Matrix<T, 3, 1> trans_W_Ci = rot_W_Cb * trans_Cb_Ci + trans_W_Cb;
    Eigen::Quaternion<T> rot_W_Ci = rot_W_Cb * rot_Cb_Ci;

    // Transform target point from target frame to world frame.
    Eigen::Matrix<T, 3, 1> target_point_3d_in_Ti = target_point3d_.cast<T>();
    Eigen::Matrix<T, 3, 1> target_point_3d_in_W =
        rot_W_Ti * target_point_3d_in_Ti + trans_W_Ti;

    // Project the 3D point to 2D image plane.
    Eigen::Matrix<T, 2, 1> projected_point_2d;
    if (auto ptr =
            std::dynamic_pointer_cast<CamRadtanIntrinsic>(cam_intrinsic_)) {
      CamRadtanIntrinsic::Space2Image(trans_W_Ci, rot_W_Ci,
                                      target_point_3d_in_W, cam_intrinsics_ptr,
                                      projected_point_2d);
    } else if (auto ptr = std::dynamic_pointer_cast<CamEqdistIntrinsic>(
                   cam_intrinsic_)) {
      CamEqdistIntrinsic::Space2Image(trans_W_Ci, rot_W_Ci,
                                      target_point_3d_in_W, cam_intrinsics_ptr,
                                      projected_point_2d);
    } else {
      // Unsupported camera model.
      return false;
    }

    // Compute residuals (reprojection error).
    Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals(residuals_ptr);
    Eigen::Matrix<T, 2, 1> observed_point_2d = iamge_point2d_.cast<T>();
    residuals = T(weight_) * (projected_point_2d - observed_point_2d);

    return true;
  }

 private:
  // Camera intrinsic that used for determining the projection model.
  CamIntrinsicBase::Ptr cam_intrinsic_;
  // Fiducial 3D corner in target local frame.
  Eigen::Vector3d target_point3d_;
  // Observed 2D keypoint in current image.
  Eigen::Vector2d iamge_point2d_;
  // Weight for the residual.
  double weight_ = 1.0;
};

}  // namespace xr_ucalib

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

// clang-format off
#include <memory>

#include <ceres/ceres.h>
#include <sophus/se3.hpp>

#include "xr_ucalib/uc_common/calib_parameter/cam_intrinsic_base.h"
// clang-format on

namespace xr_ucalib {

/**
 * @brief Camera intrinsic parameters for the equidistant distortion model.
 *
 * This implementation follows the 'equidistant' model definition used in
 * Kalibr.
 *
 * We provide a templated 'SpaceToImage' function that implements the projection
 * from 3D space to the 2D image plane. This is specifically designed to support
 * Ceres Solver's automatic differentiation
 */
class CamEqdistIntrinsic : public CamIntrinsicBase {
 public:
  using Ptr = std::shared_ptr<CamEqdistIntrinsic>;

  // Factory method to create a shared pointer instance.
  static Ptr Create() { return Ptr(new CamEqdistIntrinsic()); }

  // Accessors for camera intrinsic parameters.
  double& fx() { return parameters[0]; }
  double& fy() { return parameters[1]; }
  double& cx() { return parameters[2]; }
  double& cy() { return parameters[3]; }
  double& k1() { return parameters[4]; }
  double& k2() { return parameters[5]; }
  double& k3() { return parameters[6]; }
  double& k4() { return parameters[7]; }

  /**
   * @brief Project a 3D point in world coordinates to 2D image coordinates.
   *
   * @tparam T Template type for automatic differentiation. (e.g. double,
   * ceres::Jet)
   * @param[in] trans_W_C Translation from world to camera frame.
   * @param[in] rot_W_C Rotation from world to camera frame.
   * @param[in] point_in_W 3D point in world coordinates.
   * @param[in] params Intrinsic parameters array.
   * @param[out] point_2d Output 2D point in image coordinates.
   */
  template <typename T>
  static void Space2Image(const Eigen::Matrix<T, 3, 1>& trans_W_C,
                          const Eigen::Quaternion<T>& rot_W_C,
                          const Eigen::Matrix<T, 3, 1>& point_in_W,
                          const T* const params,
                          Eigen::Matrix<T, 2, 1>& point_2d) {
    Sophus::Vector3<T> point_in_C =
        rot_W_C.inverse() * (point_in_W - trans_W_C);

    // Extract intrinsic parameters.
    T fx = params[0];
    T fy = params[1];
    T cx = params[2];
    T cy = params[3];

    T k1 = params[4];
    T k2 = params[5];
    T k3 = params[6];
    T k4 = params[7];

    // Compute the angle theta and phi.
    T len = ceres::sqrt(point_in_C[0] * point_in_C[0] +
                        point_in_C[1] * point_in_C[1] +
                        point_in_C[2] * point_in_C[2]);
    T theta = ceres::acos(point_in_C[2] / len);
    T phi = ceres::atan2(point_in_C[1], point_in_C[0]);

    // Apply equidistant distortion model.
    Sophus::Vector2<T> p_distorted =
        DistortTheta(k1, k2, k3, k4, theta) *
        Sophus::Vector2<T>(ceres::cos(phi), ceres::sin(phi));

    // Map to pixel coordinates.
    point_2d(0) = fx * p_distorted(0) + cx;
    point_2d(1) = fy * p_distorted(1) + cy;
  }

 private:
  CamEqdistIntrinsic() {
    parameter_size = 8;  // fx, fy, cx, cy, k1, k2, k3, k4
    parameters.resize(parameter_size, 0.0);
    cam_model_type = CamModelType::EQUIDISTANT;
  }

  /// @brief Apply equidistant distortion to angle theta.
  template <typename T>
  static T DistortTheta(T k1, T k2, T k3, T k4, T theta) {
    T theta2 = theta * theta;
    T theta3 = theta2 * theta;
    T theta5 = theta3 * theta2;
    T theta7 = theta5 * theta2;
    T theta9 = theta7 * theta2;

    return theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9;
  }
};

}  // namespace xr_ucalib
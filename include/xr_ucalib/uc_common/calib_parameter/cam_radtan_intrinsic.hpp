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
 * @brief Camera intrinsic parameters for the radial-tangential distortion
 * model.
 *
 * This implementation follows the 'radtan' model definition used in kalibr.
 *
 * We provide a templated 'SpaceToImage' function that implements the projection
 * from 3D space to the 2D image plane. This is specifically designed to support
 * Ceres Solver's automatic differentiation.
 */
class CamRadtanIntrinsic : public CamIntrinsicBase {
 public:
  using Ptr = std::shared_ptr<CamRadtanIntrinsic>;

  // Factory method to create a shared pointer instance.
  static Ptr Create() { return Ptr(new CamRadtanIntrinsic()); }

  // Accessors for camera intrinsic parameters.
  double& fx() { return parameters[0]; }
  double& fy() { return parameters[1]; }
  double& cx() { return parameters[2]; }
  double& cy() { return parameters[3]; }
  double& k1() { return parameters[4]; }
  double& k2() { return parameters[5]; }
  double& p1() { return parameters[6]; }
  double& p2() { return parameters[7]; }

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
    T p1 = params[6];
    T p2 = params[7];

    // Normalize coordinates.
    T x_norm = point_in_C[0] / point_in_C[2];
    T y_norm = point_in_C[1] / point_in_C[2];

    // Apply radial and tangential distortion.
    T r2 = x_norm * x_norm + y_norm * y_norm;
    T radial_distortion = T(1.0) + k1 * r2 + k2 * r2 * r2;
    T x_distorted = x_norm * radial_distortion + T(2.0) * p1 * x_norm * y_norm +
                    p2 * (r2 + T(2.0) * x_norm * x_norm);
    T y_distorted = y_norm * radial_distortion +
                    p1 * (r2 + T(2.0) * y_norm * y_norm) +
                    T(2.0) * p2 * x_norm * y_norm;

    // Map to pixel coordinates.
    point_2d(0) = fx * x_distorted + cx;
    point_2d(1) = fy * y_distorted + cy;
  }

 private:
  CamRadtanIntrinsic() {
    cam_model_type = CamModelType::RADTAN;
    parameter_size = 8;  // fx, fy, cx, cy, k1, k2, p1, p2
    parameters.resize(parameter_size, 0.0);
  }
};

}  // namespace xr_ucalib
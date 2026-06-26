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

#include <nlohmann/json.hpp>
#include <sophus/se3.hpp>

#include "xr_ucalib/uc_common/config/json_adapter.hpp"
// clang-format on

namespace xr_ucalib {

/**
 * @brief IMU intrinsic parameters.
 *
 * The measurement model is:
 *
 * acc_meas = M_acc * acc_true + b_acc
 * gyr_meas = M_gyr * rot_ga * gyr_true + b_gyr
 *
 * M is a 3x3 upper-triangular matrix defined as:
 * | scale_x    non_ortho_xy  non_ortho_xz |
 * | 0          scale_y       non_ortho_yz |
 * | 0          0             scale_z      |
 */
struct ImuIntrinsic {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<ImuIntrinsic>;

  // Factory method to create a shared pointer instance.
  static Ptr Create() { return Ptr(new ImuIntrinsic()); }

  // Biases for IMU measurements.
  Sophus::Vector3d acc_bias = Sophus::Vector3d::Zero();
  Sophus::Vector3d gyr_bias = Sophus::Vector3d::Zero();

  // Scale factors for IMU measurements.
  Sophus::Vector3d acc_scale = Sophus::Vector3d::Ones();
  Sophus::Vector3d gyr_scale = Sophus::Vector3d::Ones();

  // Axis non-orthogonal factors for IMU measurements.
  Sophus::Vector3d acc_non_orthogonal = Sophus::Vector3d::Zero();
  Sophus::Vector3d gyr_non_orthogonal = Sophus::Vector3d::Zero();

  // Rotational misalignment from accelerometer to gyroscope.
  Sophus::SO3d rot_gyr_acc = Sophus::SO3d();

 private:
  ImuIntrinsic() = default;
};

// JSON (de)serialization support for ImuIntrinsic.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ImuIntrinsic, acc_bias,
                                                gyr_bias, acc_scale, gyr_scale,
                                                acc_non_orthogonal,
                                                gyr_non_orthogonal,
                                                rot_gyr_acc);

}  // namespace xr_ucalib

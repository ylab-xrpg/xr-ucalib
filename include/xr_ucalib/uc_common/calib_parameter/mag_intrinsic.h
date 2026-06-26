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
 * @brief Magnetometer intrinsic parameters.
 *
 * The measurement model is:
 *
 * mag_meas = soft_iron_matrix * mag_true + hard_iron_offset
 */
struct MagIntrinsic {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<MagIntrinsic>;

  // Factory method to create a shared pointer instance.
  static Ptr Create() { return Ptr(new MagIntrinsic()); }

  // Ellipsoid model parameters (a, b, c, d, e, f, g, h, i, j) used for
  // magnetometer intrinsic calibration.
  Sophus::Matrix<double, 10, 1> ellipsoid_params =
      Sophus::Matrix<double, 10, 1>::Zero();

  // Cosine of the angle between the local gravity vector and the magnetic field
  // vector.
  double cos_theta = 1.0;

  // Generalized soft-iron distortion for magnetometer measurements.
  Sophus::Matrix3d soft_iron_matrix = Sophus::Matrix3d::Identity();

  // Generalized hard-iron offset for magnetometer measurements.
  Sophus::Vector3d hard_iron_offset = Sophus::Vector3d::Zero();

 private:
  MagIntrinsic() = default;
};

// JSON (de)serialization support for MagIntrinsic.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(MagIntrinsic, ellipsoid_params,
                                                cos_theta, soft_iron_matrix,
                                                hard_iron_offset);

}  // namespace xr_ucalib

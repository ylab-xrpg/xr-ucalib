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
#include <string>

#include <Eigen/Eigen>
#include <nlohmann/json.hpp>

#include "xr_ucalib/uc_common/config/json_adapter.hpp"
#include "xr_ucalib/uc_common/config/types.h"
// clang-format on

namespace xr_ucalib {

/// @brief Magnetometer configuration used for calibration and system setup.
struct MagConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Name of the magnetometer data file, also used as a unique identifier.
  std::string file_name = "";

  // Sensor type (default: magnetometer).
  SensorType sensor_type = SensorType::MAGNETOMETER;

  // Down-sampling rate for magnetometer data in unified calibration.
  // The down-sampling rate is set independently for unified calibration to
  // allow finer control over the trade-off between accuracy and efficiency.
  int down_sample_rate_ucalib = 1;

  // Noise level of the normalized magnetometer measurement (discrete-time).
  double noise = 0.05;

  // Options indicating whether to fix calibration parameters during
  // optimization.
  bool fix_temporal_extrinsic = false;
  bool fix_spatial_extrinsic = false;
  // bool fix_intrinsic = false;

  // Priors for the spatiotemporal extrinsics (Transform from magnetometer to
  // body).
  double toff_B_Mi_prior = 0.;
  Eigen::Quaterniond rot_q_B_Mi_prior = Eigen::Quaterniond::Identity();

  // Prior calibration values for soft-iron and hard-iron corrections.
  // TODO: Now, Magnetometer data is assumed to be pre-calibrated via ellipsoid
  // fitting and normalized to unit length, so we don't include intrinsic
  // parameters for magnetometers. We can add them in the future if needed.
  // Eigen::Matrix3d soft_iron_prior = Eigen::Matrix3d::Identity();
  // Eigen::Vector3d hard_iron_prior = Eigen::Vector3d::Zero();
};

// JSON (de)serialization support for MagConfig.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(MagConfig, file_name,
                                                down_sample_rate_ucalib, noise,
                                                fix_temporal_extrinsic,
                                                fix_spatial_extrinsic,
                                                toff_B_Mi_prior,
                                                rot_q_B_Mi_prior);

}  // namespace xr_ucalib
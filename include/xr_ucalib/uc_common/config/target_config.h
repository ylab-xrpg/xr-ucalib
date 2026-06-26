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

/// @brief Fiducial target configuration used for calibration and system setup.
struct TargetConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Sensor type (default: fiducial).
  SensorType sensor_type = SensorType::FIDUCIAL;

  // Fiducial type (e.g., AprilTag).
  FiducialType fiducial_type = FiducialType::APRILTAG;

  // Index of the fiducial target, used when multiple targets are present.
  int target_idx = -1;

  // Physical size of each fiducial tag (meters).
  double fiducial_size = 0.088;
  // Ratio between tag spacing and tag size.
  double fiducial_spacing = 0.02;
  // Number of rows in the fiducial grid.
  int fiducial_rows = 6;
  // Number of columns in the fiducial grid.
  int fiducial_cols = 6;

  // Start ID for the corners of this target.
  int start_id = 0;

  // Option indicating whether to fix spatial extrinsic parameters during
  // optimization.
  bool fix_spatial_extrinsic = false;

  // Priors for spatial extrinsics (Transform from fiducial to world, we set the
  // first target as the world frame).
  Eigen::Vector3d trans_W_T_prior = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rot_q_W_T_prior = Eigen::Quaterniond::Identity();
};

// JSON (de)serialization support for TargetConfig.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(
    TargetConfig, fiducial_type, target_idx, fiducial_size, fiducial_spacing,
    fiducial_rows, fiducial_cols, start_id, fix_spatial_extrinsic,
    trans_W_T_prior, rot_q_W_T_prior);

}  // namespace xr_ucalib
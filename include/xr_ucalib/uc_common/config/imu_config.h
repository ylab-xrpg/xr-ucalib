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

/// @brief IMU configuration used for calibration and system setup.
struct ImuConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Name of the IMU data file, also used as a unique identifier.
  std::string file_name = "";

  // Sensor type (default: IMU).
  SensorType sensor_type = SensorType::IMU;

  // IMU noise/model type.
  ImuModelType imu_model_type = ImuModelType::CALIBRATED;

  // Down-sampling rate for IMU data in unified calibration.
  // The down-sampling rate is set independently for unified calibration to
  // allow finer control over the trade-off between accuracy and efficiency.
  int down_sample_rate_ucalib = 1;

  // IMU sampling frequency (Hz).
  double frequency_hz = 200.0;
  // Noise standard deviation in continuous time: [acc, acc_bias, gyr,
  // gyr_bias].
  Eigen::Vector4d noise = Eigen::Vector4d(2e-2, 5e-3, 1e-3, 5e-4);

  // Whether this IMU frame is used as the body frame.
  bool body_frame_flag = false;
  // Options indicating whether to fix calibration parameters during
  // optimization.
  bool fix_temporal_extrinsic = false;
  bool fix_spatial_extrinsic = false;

  // Priors for the spatiotemporal extrinsics (Transform from IMU to body).
  double toff_B_Ii_prior = 0.;
  Eigen::Vector3d trans_B_Ii_prior = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rot_q_B_Ii_prior = Eigen::Quaterniond::Identity();

  // Prior IMU biases.
  Eigen::Vector3d acc_bias_prior = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr_bias_prior = Eigen::Vector3d::Zero();
};

// JSON (de)serialization support for ImuConfig.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(
    ImuConfig, file_name, imu_model_type, down_sample_rate_ucalib, frequency_hz,
    noise, body_frame_flag, fix_temporal_extrinsic, fix_spatial_extrinsic,
    noise, toff_B_Ii_prior, trans_B_Ii_prior, rot_q_B_Ii_prior, acc_bias_prior,
    gyr_bias_prior);

}  // namespace xr_ucalib
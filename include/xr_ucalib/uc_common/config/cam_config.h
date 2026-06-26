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

/// @brief Camera configuration used for calibration and system setup.
struct CamConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Name of the camera data directory, also used as a unique identifier.
  std::string file_name = "";

  // Sensor type (default: camera).
  SensorType sensor_type = SensorType::CAMERA;

  // Camera model type.
  CamModelType cam_model_type = CamModelType::RADTAN;

  // Detection display mode.
  DetectionDisplayMode detection_display_mode = DetectionDisplayMode::NONE;

  // Whether to save all per-frame reprojection validation images (only used in
  // camera calibration). If false, only uniformly sampled frames will be saved.
  bool save_all_reproj_image = false;

  // Reprojection error threshold (in pixels) for outlier rejection during
  // optimization. A negative value indicates that no outlier rejection will be
  // performed.
  double reproj_threshold = -1;

  // Down-sampling rate for images in unified calibration.
  // The down-sampling rate is set independently for unified calibration to
  // allow finer control over the trade-off between accuracy and efficiency.
  int down_sample_rate_ucalib = 1;

  // Initial focal length guess. Must be specified by the user to ensure proper
  // convergence of camera calibration.
  double initial_focal_length = -1.;

  // Image width and height in pixels.
  int width = 0.;
  int height = 0.;

  /// Camera measurement noise (in pixels).
  double noise = 0.5;

  // Whether this camera is set as the base camera for multi-camera calibration.
  bool base_camera_flag = false;
  // Options indicating whether to fix calibration parameters during
  // optimization.
  bool fix_temporal_extrinsic = false;
  bool fix_spatial_extrinsic = false;
  bool fix_intrinsic = false;

  // Priors for spatiotemporal extrinsics (Transform from i-th camera to base
  // camera. If current camera is base, this is the transform from base to
  // body).
  double toff_Cb_Ci_prior = 0.;
  Eigen::Vector3d trans_Cb_Ci_prior = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rot_q_Cb_Ci_prior = Eigen::Quaterniond::Identity();

  // Intrinsic parameter priors.
  std::vector<double> intrinsic_prior = {0., 0., 0., 0.};
};

// JSON (de)serialization support for CamConfig.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(
    CamConfig, file_name, cam_model_type, detection_display_mode,
    save_all_reproj_image, reproj_threshold, down_sample_rate_ucalib,
    initial_focal_length, noise, base_camera_flag, fix_temporal_extrinsic,
    fix_spatial_extrinsic, fix_intrinsic, toff_Cb_Ci_prior, trans_Cb_Ci_prior,
    rot_q_Cb_Ci_prior, intrinsic_prior);

}  // namespace xr_ucalib
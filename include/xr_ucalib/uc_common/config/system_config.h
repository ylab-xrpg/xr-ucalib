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
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "xr_ucalib/uc_common/config/cam_config.h"
#include "xr_ucalib/uc_common/config/imu_config.h"
#include "xr_ucalib/uc_common/config/json_adapter.hpp"
#include "xr_ucalib/uc_common/config/mag_config.h"
#include "xr_ucalib/uc_common/config/target_config.h"
// clang-format on

namespace xr_ucalib {

/// @brief Camera calibration configuration.
struct CamCalibConfig {
  // Enable bundle adjustment refinement (camera rig calibration).
  bool enable_ba_refine = true;

  // Down-sampling rate for images in camera calibration.
  // Since we require synchronized images across all cameras, down-sampling
  // rate is applied to all cameras uniformly.
  int cam_down_sample_rate = 1;

  // Number of threads for multi-threaded processing, such as feature extraction
  // and matching, ceres optimization, etc.
  int multi_thread_num = 8;

  // Ceres solver settings.
  int ceres_max_iterations = 20;
};

/// @brief Unified multi-sensor calibration configuration.
struct UnifiedCalibConfig {
  // Whether to perform continuous-time unified calibration.
  bool enable_unified_calib = false;

  // Spline order for continuous-time trajectory representation.
  static constexpr int kSplineOrder = 4;

  // Interval between spline knots (seconds).
  double spline_knot_interval = 0.01;
  // Maximum allowed change of time offset during optimization.
  double max_toff_change = 0.1;

  // Gravity magnitude in m/s^2.
  double gravity_magnitude = 9.8;

  // Whether to fix camera parameters in unified calibration.
  bool fix_camera_intrinsics = true;
  bool fix_camera_extrinsics = true;

  // Number of threads for multi-threaded processing, such as PnP initialization
  // for base camera trajectory, ceres optimization, etc.
  int multi_thread_num = 8;

  // Ceres solver settings
  int ceres_max_iterations = 20;
};

/**
 * @brief System-wide configuration, including all sensors and calibration
 * settings.
 */
class SystemConfig {
 public:
  using Ptr = std::shared_ptr<SystemConfig>;

  // Factory method to create a shared pointer instance.
  static Ptr Create() { return Ptr(new SystemConfig()); }

  // Load configuration from a JSON file (deserialize).
  bool FromJson(const std::string &input_path);

  // Save configuration to a JSON file (Serialize).
  bool ToJson(const std::string &output_path);

  // Verify configuration and print key information.
  bool CheckAndPrintConfig();

  // Camera calibration settings.
  CamCalibConfig cam_calib_config;
  // Unified multi-sensor calibration settings.
  UnifiedCalibConfig unified_calib_config;

  // Configurations of all cameras in the system.
  std::vector<CamConfig> cam_configs;
  // Configurations of all IMUs in the system.
  std::vector<ImuConfig> imu_configs;
  // Configurations of all magnetometers in the system.
  std::vector<MagConfig> mag_configs;
  // Configurations of all fiducial targets in the system.
  std::vector<TargetConfig> target_configs;

  // Paths for sensor data and workspace. These are not loaded from JSON but set
  // from command-line arguments in the main application.
  std::string sensor_data_dir = "";
  std::string workspace_dir = "";

 private:
  SystemConfig() = default;
};

// TODO: Use intrusive serialization to avoid exposing configuration members
// publicly.
// JSON (de)serialization support for CamCalibConfig.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(CamCalibConfig,
                                                enable_ba_refine,
                                                cam_down_sample_rate,
                                                multi_thread_num,
                                                ceres_max_iterations);

// JSON (de)serialization support for UnifiedCalibConfig.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(
    UnifiedCalibConfig, enable_unified_calib, spline_knot_interval,
    max_toff_change, gravity_magnitude, fix_camera_intrinsics,
    fix_camera_extrinsics, multi_thread_num, ceres_max_iterations);

// JSON (de)serialization support for SystemConfig.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SystemConfig, cam_calib_config,
                                                unified_calib_config,
                                                cam_configs, imu_configs,
                                                mag_configs, target_configs);

}  // namespace xr_ucalib
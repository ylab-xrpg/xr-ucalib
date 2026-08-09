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
#include "xr_ucalib/uc_common/config/system_config.h"

#include <cmath>
#include <fstream>
#include <vector>

#include <spdlog/spdlog.h>
// clang-format on

namespace xr_ucalib {
namespace {

bool IsPositiveFinite(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool IsNonNegativeFinite(double value) {
  return std::isfinite(value) && value >= 0.0;
}

}  // namespace

bool SystemConfig::FromJson(const std::string& input_path) {
  spdlog::info("Read system config from: {}", input_path);

  try {
    std::ifstream input_file(input_path);
    if (!input_file.is_open()) {
      spdlog::error("Failed to open input JSON file: {}", input_path);
      return false;
    }

    nlohmann::json nlm_json;
    input_file >> nlm_json;
    input_file.close();

    // Deserialize.
    nlm_json.get_to(*this);

    if (!CheckAndPrintConfig()) {
      spdlog::error("Configuration validation failed.");
      return false;
    }

  } catch (const std::exception& e) {
    spdlog::error("JSON parsing error: {}", e.what());
    return false;
  } catch (...) {
    spdlog::error("Unknown error reading JSON.");
    return false;
  }

  return true;
}

bool SystemConfig::ToJson(const std::string& output_path) {
  spdlog::info("Write system config to: {}", output_path);

  try {
    std::ofstream output_file(output_path);
    if (!output_file.is_open()) {
      spdlog::error("Failed to open output JSON: {}", output_path);
      return false;
    }

    nlohmann::json nlm_json;
    nlm_json = *this;

    // Serialize.
    output_file << nlm_json.dump(2);
    output_file.flush();
    if (!output_file.good()) {
      spdlog::error("Failed while writing system configuration: {}",
                    output_path);
      return false;
    }

  } catch (const std::exception& e) {
    spdlog::error("JSON writing error: {}", e.what());
    return false;
  } catch (...) {
    spdlog::error("Unknown error writing JSON.");
    return false;
  }

  return true;
}

bool SystemConfig::CheckAndPrintConfig() {
  // Step 1: Validate system-wide numerical settings before they are used in
  // down-sampling, thread creation, or optimization.
  if (cam_calib_config.cam_down_sample_rate <= 0) {
    spdlog::error("Camera calibration down-sample rate must be positive.");
    return false;
  }
  if (cam_calib_config.multi_thread_num <= 0) {
    spdlog::error("Camera calibration thread count must be positive.");
    return false;
  }
  if (cam_calib_config.ceres_max_iterations <= 0) {
    spdlog::error("Camera calibration Ceres iteration count must be positive.");
    return false;
  }

  if (unified_calib_config.enable_unified_calib) {
    if (!IsPositiveFinite(unified_calib_config.spline_knot_interval)) {
      spdlog::error("Spline knot interval must be finite and positive.");
      return false;
    }
    if (!IsNonNegativeFinite(unified_calib_config.max_toff_change)) {
      spdlog::error(
          "Maximum time-offset change must be finite and non-negative.");
      return false;
    }
    if (!IsPositiveFinite(unified_calib_config.gravity_magnitude)) {
      spdlog::error("Gravity magnitude must be finite and positive.");
      return false;
    }
    if (unified_calib_config.multi_thread_num <= 0) {
      spdlog::error("Unified calibration thread count must be positive.");
      return false;
    }
    if (unified_calib_config.ceres_max_iterations <= 0) {
      spdlog::error(
          "Unified calibration Ceres iteration count must be positive.");
      return false;
    }
  }

  // Step 2: Determine calibration mode.
  if (unified_calib_config.enable_unified_calib) {
    spdlog::info(
        "Calibration mode: Unified multi-sensor. Camera -> joint multi-sensor "
        "calibration.");

    if (imu_configs.empty()) {
      spdlog::error(
          "At least one IMU must be configured for unified calibration.");

      return false;
    }
  } else {
    spdlog::info("Calibration mode: Camera-only.");

    if (!imu_configs.empty() || !mag_configs.empty()) {
      spdlog::warn(
          "IMU or magnetometer configurations found but unified calibration "
          "is disabled. They will be ignored.");
    }
  }

  if (cam_configs.empty() || target_configs.empty()) {
    spdlog::error("At least one camera and one target must be configured.");

    return false;
  }

  spdlog::info("----------------- System Configuration ----------------");

  // ===========================================================================

  // Step 3: Print system config.
  spdlog::info(" - Camera Calibration Config:");
  spdlog::info("Whether to enable bundle adjustment refinement: {}",
               cam_calib_config.enable_ba_refine ? "Yes" : "No");
  spdlog::info("Frame down-sample rate for camera calibration: {}",
               cam_calib_config.cam_down_sample_rate);
  spdlog::info("Camera calibration thread count: {}",
               cam_calib_config.multi_thread_num);
  spdlog::info("Ceres maximum iterations: {}",
               cam_calib_config.ceres_max_iterations);

  if (unified_calib_config.enable_unified_calib) {
    spdlog::info(" - Unified Calibration Config:");
    spdlog::info("Spline knot interval: {:.3f} s",
                 unified_calib_config.spline_knot_interval);
    spdlog::info("Max time offset change in optimization: {:.3f} s",
                 unified_calib_config.max_toff_change);
    spdlog::info("Gravity magnitude: {:.3f} m/s^2",
                 unified_calib_config.gravity_magnitude);
    spdlog::info("Whether to fix camera intrinsics: {}",
                 unified_calib_config.fix_camera_intrinsics ? "Yes" : "No");
    spdlog::info("Whether to fix camera extrinsics: {}",
                 unified_calib_config.fix_camera_extrinsics ? "Yes" : "No");
    spdlog::info("Unified calibration thread count: {}",
                 unified_calib_config.multi_thread_num);
    spdlog::info("Ceres maximum iterations: {}",
                 unified_calib_config.ceres_max_iterations);
  }

  // ===========================================================================

  // Step 4: Validate and print sensor configs.
  spdlog::info("---------------- Sensor Configurations ----------------");

  int body_frame_count = 0;
  int base_camera_count = 0;

  // Step 4.1: Print camera configs.
  for (const auto& cam_config : cam_configs) {
    if (cam_config.file_name == "") {
      spdlog::error("Camera config found with empty file_name.");
      return false;
    } else {
      spdlog::info(" - Camera: {}", cam_config.file_name);
    }

    if (cam_config.cam_model_type == CamModelType::INVALID) {
      spdlog::error("Unsupported camera model type: {}",
                    CamModelTypeToString(cam_config.cam_model_type));
      return false;
    }
    spdlog::info("Camera model type: {}.",
                 CamModelTypeToString(cam_config.cam_model_type));

    spdlog::info("Save all per-frame reprojection images: {}",
                 cam_config.save_all_reproj_image ? "Yes" : "No");

    spdlog::info("Reprojection error threshold: {:.3f} pixels.",
                 cam_config.reproj_threshold);

    if (!std::isfinite(cam_config.reproj_threshold)) {
      spdlog::error("Camera reprojection threshold must be finite.");
      return false;
    }

    if (cam_config.down_sample_rate_ucalib <= 0) {
      spdlog::error(
          "Unified calibration down-sample rate for camera {} must be "
          "positive.",
          cam_config.file_name);
      return false;
    }

    if (unified_calib_config.enable_unified_calib) {
      spdlog::info("Down-sample rate for unified calibration: {}",
                   cam_config.down_sample_rate_ucalib);
    }

    if (!IsPositiveFinite(cam_config.initial_focal_length)) {
      spdlog::error(
          "Initial focal length for camera {} must be finite and positive.",
          cam_config.file_name);
      return false;
    } else {
      spdlog::info("Initial focal length: {:.1f} pixels.",
                   cam_config.initial_focal_length);
    }

    if (!IsPositiveFinite(cam_config.noise)) {
      spdlog::error(
          "Measurement noise for camera {} must be finite and positive.",
          cam_config.file_name);
      return false;
    }
    spdlog::info("Noise standard deviation: {:.3f} pixels.", cam_config.noise);

    if (cam_config.base_camera_flag) {
      spdlog::info("Set as base camera.");
      base_camera_count++;
    }

    if (cam_config.fix_temporal_extrinsic) {
      spdlog::info("Fix temporal extrinsic to prior in calibration.");
    }
    if (cam_config.fix_spatial_extrinsic) {
      spdlog::info("Fix spatial extrinsic to prior in calibration.");
    }
    if (cam_config.fix_intrinsic) {
      spdlog::info("Fix intrinsic to prior in calibration.");
    }
  }

  // ===========================================================================

  // Step 4.2: Print IMU configs.
  for (const auto& imu_config : imu_configs) {
    if (!unified_calib_config.enable_unified_calib) break;

    if (imu_config.file_name == "") {
      spdlog::error("IMU config found with empty file_name.");
      return false;
    } else {
      spdlog::info(" - IMU: {}", imu_config.file_name);
    }

    spdlog::info("IMU model type: {}.",
                 ImuModelTypeToString(imu_config.imu_model_type));

    if (imu_config.imu_model_type == ImuModelType::INVALID) {
      spdlog::error("Unsupported IMU model type for {}.", imu_config.file_name);
      return false;
    }

    if (imu_config.down_sample_rate_ucalib <= 0) {
      spdlog::error(
          "Unified calibration down-sample rate for IMU {} must be positive.",
          imu_config.file_name);
      return false;
    }

    if (!IsPositiveFinite(imu_config.frequency_hz)) {
      spdlog::error("Frequency for IMU {} must be finite and positive.",
                    imu_config.file_name);
      return false;
    }

    if (!imu_config.noise.allFinite() ||
        (imu_config.noise.array() <= 0.0).any()) {
      spdlog::error("All noise values for IMU {} must be finite and positive.",
                    imu_config.file_name);
      return false;
    }

    spdlog::info("Down-sample rate for unified calibration: {}",
                 imu_config.down_sample_rate_ucalib);

    spdlog::info("IMU frequency: {:.1f} Hz.", imu_config.frequency_hz);
    spdlog::info(
        "Noise standard deviations [acc_n, acc_b, gyr_n, gyr_b]: "
        "[{:.6f}, {:.6f}, "
        "{:.6f}, {:.6f}].",
        imu_config.noise[0], imu_config.noise[1], imu_config.noise[2],
        imu_config.noise[3]);

    if (imu_config.body_frame_flag) {
      spdlog::info("Set as body frame.");
      body_frame_count++;
    }
    if (imu_config.fix_temporal_extrinsic) {
      spdlog::info("Fix temporal extrinsic to prior in calibration.");
    }
    if (imu_config.fix_spatial_extrinsic) {
      spdlog::info("Fix spatial extrinsic to prior in calibration.");
    }
  }

  // ===========================================================================

  // Step 4.3: Print magnetometer configs.
  for (const auto& mag_config : mag_configs) {
    if (!unified_calib_config.enable_unified_calib) break;

    if (mag_config.file_name == "") {
      spdlog::error("Magnetometer config found with empty file_name.");
      return false;
    } else {
      spdlog::info(" - Magnetometer: {}", mag_config.file_name);
    }

    spdlog::info("Down-sample rate for unified calibration: {}",
                 mag_config.down_sample_rate_ucalib);

    if (mag_config.down_sample_rate_ucalib <= 0) {
      spdlog::error(
          "Unified calibration down-sample rate for magnetometer {} must be "
          "positive.",
          mag_config.file_name);
      return false;
    }

    if (!IsPositiveFinite(mag_config.noise)) {
      spdlog::error(
          "Measurement noise for magnetometer {} must be finite and positive.",
          mag_config.file_name);
      return false;
    }

    spdlog::info("Noise standard deviation: {:.3f}.", mag_config.noise);

    if (mag_config.fix_temporal_extrinsic) {
      spdlog::info("Fix temporal extrinsic to prior in calibration.");
    }
    if (mag_config.fix_spatial_extrinsic) {
      spdlog::info("Fix spatial extrinsic to prior in calibration.");
    }
    // if (mag_config.fix_intrinsic) {
    //   spdlog::info("Fix intrinsic to prior in calibration.");
    // }
  }

  // ===========================================================================

  // Step 4.4: Print fiducial target configs.
  bool is_first_target = true;
  FiducialType common_fiducial_type;

  for (const auto& target_config : target_configs) {
    if (target_config.target_idx < 0) {
      spdlog::error("Target config found with invalid target_idx: {}",
                    target_config.target_idx);
      return false;
    } else {
      spdlog::info(" - Target: {}", target_config.target_idx);
    }

    if (is_first_target) {
      common_fiducial_type = target_config.fiducial_type;
      is_first_target = false;
    } else {
      if (target_config.fiducial_type != common_fiducial_type) {
        spdlog::error("All targets must have the same fiducial type.");
        return false;
      }
    }

    spdlog::info("Fiducial type: {}.",
                 FiducialTypeToString(target_config.fiducial_type));

    if (target_config.fiducial_type == FiducialType::INVALID) {
      spdlog::error("Unsupported fiducial type for target {}.",
                    target_config.target_idx);
      return false;
    }

    if (!IsPositiveFinite(target_config.fiducial_size)) {
      spdlog::error("Fiducial size for target {} must be finite and positive.",
                    target_config.target_idx);
      return false;
    }
    if (!IsNonNegativeFinite(target_config.fiducial_spacing)) {
      spdlog::error(
          "Fiducial spacing for target {} must be finite and non-negative.",
          target_config.target_idx);
      return false;
    }
    if (target_config.fiducial_rows <= 0 || target_config.fiducial_cols <= 0) {
      spdlog::error("Fiducial grid dimensions for target {} must be positive.",
                    target_config.target_idx);
      return false;
    }

    spdlog::info("Fiducial size: {:.3f} m.", target_config.fiducial_size);
    spdlog::info("Fiducial spacing: {:.3f} ratio.",
                 target_config.fiducial_spacing);
    spdlog::info("Fiducial rows: {}.", target_config.fiducial_rows);
    spdlog::info("Fiducial cols: {}.", target_config.fiducial_cols);

    if (target_config.fix_spatial_extrinsic) {
      spdlog::info("Fix spatial extrinsic to prior in calibration.");
    }
  }

  // ===========================================================================

  // Step 5: Final checks.
  if (body_frame_count != 1 && unified_calib_config.enable_unified_calib) {
    spdlog::error("Exactly one IMU must be set as body frame. Found {}.",
                  body_frame_count);
    return false;
  }

  if (base_camera_count != 1) {
    spdlog::error("Exactly one camera must be set as base camera. Found {}.",
                  base_camera_count);
    return false;
  }

  if (!unified_calib_config.enable_unified_calib) {
    spdlog::info("Summary: cameras: {}, targets: {}.", cam_configs.size(),
                 target_configs.size());
  } else {
    spdlog::info(
        "Summary: cameras: {}, IMUs: {}, magnetometers: {}, targets: {}.",
        cam_configs.size(), imu_configs.size(), mag_configs.size(),
        target_configs.size());
  }

  // ===========================================================================

  return true;
}

}  // namespace xr_ucalib

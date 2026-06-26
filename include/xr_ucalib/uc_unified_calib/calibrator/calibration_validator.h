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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "xr_ucalib/uc_common/calib_parameter/calib_parameters.h"
#include "xr_ucalib/uc_common/config/system_config.h"
#include "xr_ucalib/uc_common/sensor_data/sensor_manager.h"
#include "xr_ucalib/uc_unified_calib/spline/spline_bundle.hpp"

namespace xr_ucalib {

/// @brief Validation helper for unified calibration quality checks and reports.
class CalibrationValidator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CalibrationValidator>;

  // Spline configuration.
  static int constexpr kSplineOrder = UnifiedCalibConfig::kSplineOrder;
  using SplineBundleType = SplineBundle<kSplineOrder>;
  using SplineMetaType = SplineMeta<kSplineOrder>;

  // Context to hold all shared data.
  struct Context {
    SystemConfig::Ptr system_config;
    SensorManager::Ptr sensor_manager;
    CalibParameters::Ptr calib_parameters;

    SplineBundleType::Ptr spline_bundle;
    SplineInfo trans_spline_info;
    SplineInfo rot_spline_info;
  };

  // Factory method to create a shared pointer instance.
  static Ptr Create(const Context& context) {
    return Ptr(new CalibrationValidator(context));
  }

  /**
   * @brief Calculate physically meaningful IMU residual magnitudes for one
   * frame using current calibration parameters.
   *
   * This API evaluates the same IMU accelerometer and gyroscope cost models
   * as optimization, but with unit weights (weight = 1.0), so the returned
   * residuals keep physical units.
   *
   * @param[in] label Sensor label of the IMU.
   * @param[in] imu_frame IMU frame containing measurements.
   * @param[out] acc_res_norm L2 norm of accelerometer residual.
   * @param[out] gyr_res_norm L2 norm of gyroscope residual.
   * @return true If residuals are successfully evaluated.
   */
  bool CalImuResidual(const std::string& label, const ImuFrame::Ptr& imu_frame,
                      double& acc_res_norm, double& gyr_res_norm);

  /**
   * @brief Calculate camera reprojection residual vectors for one frame using
   * current calibration parameters.
   *
   * @param[in] label Sensor label of the camera.
   * @param[in] cam_frame Camera frame containing observations.
   * @param[out] residual_pairs Reprojection residual vectors (dx, dy) for all
   * valid keypoints in this frame, in pixels.
   * @param[out] observed_points Observed keypoints (u, v), in pixels.
   * @param[out] reprojected_points Reprojected keypoints (u, v), in pixels.
   * @return true If at least one valid reprojection residual is evaluated.
   */
  bool CalCamReprojResidual(
      const std::string& label, const CamFrame::Ptr& cam_frame,
      std::vector<std::pair<double, double>>& residual_pairs,
      std::vector<std::pair<double, double>>& observed_points,
      std::vector<std::pair<double, double>>& reprojected_points);

  /**
   * @brief Validate calibrated results and save a text report.
   *
   * @param[in] output_path Path to the validation report txt file.
   * @param[in] sequence_timestamps Timestamps used in residual construction
   * for each sensor sequence.
   * @param[in] eval_start_time Evaluation start time.
   * @param[in] eval_end_time Evaluation end time.
   * @param[in] solution_usable Whether optimization solution is usable.
   * @return true If report is generated successfully.
   */
  bool ValidateAndSaveResults(
      const std::string& output_path,
      const std::map<std::string, std::vector<double>>& sequence_timestamps,
      double eval_start_time, double eval_end_time, bool solution_usable);

 private:
  explicit CalibrationValidator(const Context& context) : context_(context) {}

  /// @brief Calculate the minimum and maximum times needed for spline
  /// evaluation for a given measurement timestamp.
  bool CalMetaMinMaxTime(const double& meas_time, double& min_time,
                         double& max_time);

  // Context holding all configuration and state resources.
  Context context_;
};

}  // namespace xr_ucalib

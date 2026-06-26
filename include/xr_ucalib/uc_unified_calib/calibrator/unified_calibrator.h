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
#include <map>
#include <string>
#include <vector>

#include <ceres/ceres.h>

#include "xr_ucalib/uc_common/calib_parameter/calib_parameters.h"
#include "xr_ucalib/uc_common/config/system_config.h"
#include "xr_ucalib/uc_common/sensor_data/sensor_manager.h"
#include "xr_ucalib/uc_unified_calib/spline/spline_bundle.hpp"
// clang-format on

namespace xr_ucalib {

/**
 * @brief Unified heterogeneous multi-sensor calibrator based on continuous-time
 * representation (cubic B-spline).
 *
 * This calibrator now supports calibration of multiple cameras, IMUs,
 * Magnetometers, and fiducial targets. We first perform a linear
 * initialization, followed by continuous-time nonlinear joint optimization.
 * Benefiting from the continuous-time parameterization, our calibrator can
 * jointly optimize spatial and temporal calibration parameters, and supports
 * sensors with different sampling rates and time offsets.
 *
 * Note that:
 * 1. Individual camera calibration is required prior to the unified
 * calibration to provide reliable initial guesses for camera parameters.
 * 2. Although temporal calibration is supported, all sensors are still required
 * to share a common clock. The calibrator can only handle small temporal
 * offsets, which are typically caused by sensor-specific delays and system
 * latency.
 */
class UnifiedCalibrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<UnifiedCalibrator>;

  // Factory method to create a shared pointer instance.
  static Ptr Create(const SystemConfig::Ptr& system_config,
                    const SensorManager::Ptr& sensor_manager,
                    const CalibParameters::Ptr& calib_parameters) {
    return Ptr(
        new UnifiedCalibrator(system_config, sensor_manager, calib_parameters));
  }

  // Spline configuration.
  static int constexpr kSplineOrder = UnifiedCalibConfig::kSplineOrder;
  using SplineBundleType = SplineBundle<kSplineOrder>;
  using SplineMetaType = SplineMeta<kSplineOrder>;

  /**
   * @brief Run the unified multi-sensor calibration process.
   *
   * A linear initialization is performed first to obtain initial estimates,
   * followed by continuous-time nonlinear joint optimization.
   *
   * @return true If the calibration process is successful.
   */
  bool RunCalibration();

 private:
  UnifiedCalibrator(const SystemConfig::Ptr& system_config,
                    const SensorManager::Ptr& sensor_manager,
                    const CalibParameters::Ptr& calib_parameters)
      : system_config_(system_config),
        sensor_manager_(sensor_manager),
        calib_parameters_(calib_parameters) {}

  /**
   * @brief Initialize the states for unified calibration.
   *
   * We perform a pose-centered initialization to estimate the spatial
   * extrinsics and construct the system B-splines. The workflow includes:
   * 1. Obtain the full trajectory of the base camera using PnP.
   * 2. Solve the extrinsics between the base camera and other sensors based
   * on the base camera trajectory.
   * 3. Construct the continuous-time B-splines for the base camera trajectory
   * according to the base trajectory and spline settings.
   *
   * Some calibration parameters with small magnitudes (e.g., temporal offsets
   * and IMU biases) are set to default values and directly optimized during
   * nonlinear optimization.
   *
   * @return true If initialization is successful.
   */
  bool Initialize();

  /**
   * @brief Build and optimize the Ceres problem for unified calibration.
   *
   * We add various residuals from different sensors into the Ceres problem,
   * and then solve the optimization problem to refine calibration parameters.
   *
   * @return true If the problem is successfully built and optimized.
   */
  bool BuildAndOptimizeCereProblem();

  /**
   * @brief Initialize the base camera trajectory using PnP.
   *
   * @param[in] base_camera_label Label of the base camera.
   * @param[out] base_cam_traj Output base camera trajectory.
   * @return true If initialization is successful.
   */
  bool InitializeBaseTrajectory(std::string base_camera_label,
                                PoseSequence::Ptr& base_cam_traj);

  /**
   * @brief Get the target pose at a specified timestamp from a pose sequence.
   *
   * @param[in] pose_seq Pose sequence.
   * @param[in] timestamp Target timestamp.
   * @param[out] trans Translation part of the result pose.
   * @param[out] rot_q Rotation part of the result pose.
   * @return true If the target pose is successfully retrieved.
   */
  bool GetTargetPose(const PoseSequence::Ptr pose_seq, const double& timestamp,
                     Eigen::Vector3d& trans, Eigen::Quaterniond& rot_q);

  /**
   * @brief Save the calibrated system trajectory (B-spline) to a file.
   *
   * The trajectory is denoted as the transformation from body frame (B) to
   * world frame (W), the form of each trajectory point is [timestamp(nsec)
   * trans_W_B (x y z) rot_q_W_B (x y z w)].
   *
   * @param[in] output_path File path to save the trajectory.
   * @return true If the trajectory is successfully saved.
   */
  bool SaveSystemTrajectory(const std::string& output_path);

  /**
   * @brief Obtain the calibrated target corner positions in the world frame (W)
   * and save to a file.
   *
   * The format of each line in the output file is [target_idx corner_id x y z],
   * where target_idx is the index of the target, corner_id is the ID of the
   * corner, and x y z are the coordinates of the corner in the world frame.
   *
   * @param[in] output_path File path to save the target corners.
   * @return true If the target corners are successfully saved.
   */
  bool SaveTargetCorners(const std::string& output_path);

  /// @brief Print the calibration results after optimization.
  void PrintCalibrationResults();

  // Pointers to system configuration, sensor manager, and calibration
  // parameters.
  SystemConfig::Ptr system_config_;
  SensorManager::Ptr sensor_manager_;
  CalibParameters::Ptr calib_parameters_;

  // Spline information and spline bundle for the system continuous-time states.
  // The spline trajectory represents the transformation from body frame (B) to
  // world frame (W).
  SplineInfo trans_spline_info_, rot_spline_info_;
  SplineBundleType::Ptr spline_bundle_;
};

}  // namespace xr_ucalib
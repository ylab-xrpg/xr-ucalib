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
#include <map>
#include <memory>

#include <Eigen/Eigen>
#include <opencv2/core/types.hpp>

#include "xr_ucalib/uc_cam_calib/cam_rig_calib/reproj_evaluator.h"
#include "xr_ucalib/uc_common/calib_parameter/calib_parameters.h"
#include "xr_ucalib/uc_common/config/system_config.h"
#include "xr_ucalib/uc_common/sensor_data/sensor_manager.h"
// clang-format on

namespace xr_ucalib {

/**
 * @brief Camera rig calibrator that refines multi-camera and multi-board
 * calibration based on the initial SFM results.
 *
 * We perform non-linear optimization to minimize the reprojection errors of all
 * detected keypoints across all cameras. Based on the system configuration, a
 * base camera is selected and the reference target defines the world coordinate
 * frame. The optimization variables include:
 *  - Extrinsic parameters between each camera and the base camera.
 *  - Camera poses at each base-camera frame.
 *  - Transformations between multiple calibration targets.
 *
 * Note that:
 * 1. This calibrator can be executed independently after the initial SFM
 * calibration to perform camera-only refinement. Afterwards, camera-related
 * parameters can be fixed in the subsequent unified continuous-time calibration
 * to ensure accuracy.
 * 2. Since this module relies on discrete-time nonlinear optimization, it
 * requires strictly synchronized images across all cameras. If image
 * correspondence cannot be guaranteed, or if time offsets exist between
 * cameras, a continuous-time unified calibration should be performed directly
 * after SFM calibration.
 */
class CamRigCalibrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CamRigCalibrator>;

  // Factory method to create a shared pointer instance.
  static Ptr Create(const SystemConfig::Ptr& system_config,
                    const SensorManager::Ptr& sensor_manager,
                    const CalibParameters::Ptr& calib_parameters) {
    return Ptr(
        new CamRigCalibrator(system_config, sensor_manager, calib_parameters));
  }

  /**
   * @brief Perform camera rig calibration.
   *
   * Workflow:
   * 1. Build camera frame correspondences among all cameras based on
   * timestamps.
   * 2. Initialize camera poses for all base camera frames using PnP, which will
   * serve as the initial guess for non-linear optimization.
   * 3. Build and optimize the Ceres problem to achieve camera rig calibration.
   * 4. Compute and evaluate reprojection errors for all keypoints across all
   * frames.
   *
   * @return true If calibration is successful, false otherwise.
   */
  bool RunCalibration();

 private:
  CamRigCalibrator(const SystemConfig::Ptr& system_config,
                   const SensorManager::Ptr& sensor_manager,
                   const CalibParameters::Ptr& calib_parameters)
      : system_config_(system_config),
        sensor_manager_(sensor_manager),
        calib_parameters_(calib_parameters) {
    reproj_evaluator_ =
        ReprojEvaluator::Create(sensor_manager, calib_parameters);
  }

  // Type definitions.
  // Used to hold the pose of base camera in calibration process. Key - based
  // camera frame id, Value - (translation, rotation).
  using CamPoses =
      std::map<int, std::pair<Eigen::Vector3d, Eigen::Quaterniond>>;
  // Used to hold the corresponding frames of all cameras. Key - base camera
  // frame id, Value - map of (camera label -> frame pointer).
  using FrameCorrespondence =
      std::map<int, std::map<std::string, CamFrame::Ptr>>;
  // Used to hold the reprojection errors per CamFrame.
  // Key: camera label, Value: map of (frame pointer -> list of residuals in
  // that frame).
  using ReprojErrorPerFrame =
      std::map<std::string, std::map<CamFrame::Ptr, std::vector<cv::Point2d>>>;

  /**
   * @brief Build camera frame correspondences among all cameras in the rig.
   *
   * Note that our discrete-time camera rig calibration requires that all
   * cameras capture frames at the same timestamps. Therefore, for each base
   * camera frame, we search for frames from other cameras that have the same
   * timestamp (within a small tolerance).
   *
   * @param[out] frame_correspondence Output frame correspondences. It contains
   * frames from all cameras that correspond to the same timestamp (base camera
   * time).
   * @return true If correspondences are built successfully, false otherwise.
   */
  bool BuildCamFrameCorrespondences(FrameCorrespondence& frame_correspondence);

  /**
   * @brief Initialize camera poses for all base camera frames using PnP.
   *
   * The initialized camera poses are served as the initial guess for the
   * non-linear optimization in camera rig calibration.
   *
   * @param[in] frame_correspondence Frame correspondences among all cameras.
   * @param[out] cam_poses_W_Cb Output camera poses of the base camera
   * (Transform from base camera to world).
   * @return true If initialization is successful, false otherwise.
   */
  bool InitializeCamPoses(const FrameCorrespondence& frame_correspondence,
                          CamPoses& cam_poses_W_Cb);

  /**
   * @brief Build and optimize the Ceres problem for camera rig calibration.
   *
   * @param frame_correspondence Frame correspondences among all cameras.
   * @param cam_poses_W_Cb Initial guesses of camera poses of the base camera.
   * (Transform from base camera to world).
   * @return true If optimization is successful, false otherwise.
   */
  bool BuildAndOptimizeCeresProblem(
      const FrameCorrespondence& frame_correspondence,
      CamPoses& cam_poses_W_Cb);

  /// @brief Print the calibration results after SFM calibration.
  void PrintCalibrationResults();

  // Pointers to system configuration, sensor manager, and calibration
  // parameters.
  SystemConfig::Ptr system_config_;
  SensorManager::Ptr sensor_manager_;
  CalibParameters::Ptr calib_parameters_;

  // Reprojection evaluator.
  ReprojEvaluator::Ptr reproj_evaluator_;
};

}  // namespace xr_ucalib

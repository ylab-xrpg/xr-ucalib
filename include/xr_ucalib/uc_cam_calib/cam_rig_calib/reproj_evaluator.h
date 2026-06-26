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
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <opencv2/core/types.hpp>

#include "xr_ucalib/uc_common/calib_parameter/calib_parameters.h"
#include "xr_ucalib/uc_common/config/system_config.h"
#include "xr_ucalib/uc_common/sensor_data/sensor_manager.h"
#include "xr_ucalib/uc_common/sensor_data/sensor_data_types.h"
// clang-format on

namespace xr_ucalib {

/**
 * @brief Evaluator for camera rig calibration reprojection errors.
 */
class ReprojEvaluator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<ReprojEvaluator>;

  // Factory method to create a shared pointer instance.
  static Ptr Create(const SensorManager::Ptr& sensor_manager,
                    const CalibParameters::Ptr& calib_parameters) {
    return Ptr(new ReprojEvaluator(sensor_manager, calib_parameters));
  }

  // Type definitions (Moved from CamRigCalibrator).
  using CamPoses =
      std::map<int, std::pair<Eigen::Vector3d, Eigen::Quaterniond>>;
  using FrameCorrespondence =
      std::map<int, std::map<std::string, CamFrame::Ptr>>;
  using ReprojErrorPerFrame =
      std::map<std::string, std::map<CamFrame::Ptr, std::vector<cv::Point2d>>>;

  /**
   * @brief Compute reprojection errors for all keypoints across all frames.
   *
   * @param[in] cam_poses_W_Cb Camera poses of the base camera.
   * @param[in] frame_correspondence Frame correspondences among all cameras.
   * @param[out] reproj_errors Output container for reprojection errors.
   * @return true If successful, false otherwise.
   */
  bool ComputeReprojError(const CamPoses& cam_poses_W_Cb,
                          const FrameCorrespondence& frame_correspondence,
                          ReprojErrorPerFrame& reproj_errors);

  /**
   * @brief Remove keypoints with reprojection error exceeding the threshold.
   *
   * For each camera, if its reproj_threshold >= 0 in camera config, keypoints
   * whose reprojection error magnitude exceeds the threshold will be removed
   * from the corresponding CamFrame.
   *
   * @param[in] cam_poses_W_Cb Camera poses of the base camera.
   * @param[in] frame_correspondence Frame correspondences among all cameras.
   * @return Number of outlier keypoints removed. Returns 0 if no outliers.
   */
  size_t RemoveReprojOutliers(const CamPoses& cam_poses_W_Cb,
                              const FrameCorrespondence& frame_correspondence);

  /**
   * @brief Evaluate, plot and save reprojection error statistics.
   *
   * @param[in] reproj_errors Computed reprojection errors.
   * @param[in] work_dir Directory to save reprojection error plots and data.
   * @return true If successful, false otherwise.
   */
  bool EvaluateReprojError(const ReprojErrorPerFrame& reproj_errors,
                           const std::string& work_dir);

  /**
   * @brief Save images with observed and reprojected keypoints overlayed.
   *
   * @param[in] cam_poses_W_Cb Camera poses of the base camera.
   * @param[in] frame_correspondence Frame correspondences among all cameras.
   * @param[in] work_dir Directory to save the images.
   * @return true If successful, false otherwise.
   */
  bool SaveReprojImage(const CamPoses& cam_poses_W_Cb,
                       const FrameCorrespondence& frame_correspondence,
                       const std::string& work_dir);

 private:
  ReprojEvaluator(const SensorManager::Ptr& sensor_manager,
                  const CalibParameters::Ptr& calib_parameters)
      : sensor_manager_(sensor_manager), calib_parameters_(calib_parameters) {}

  // Pointers to sensor manager and calibration parameters.
  // parameters.
  SensorManager::Ptr sensor_manager_;
  CalibParameters::Ptr calib_parameters_;
};

}  // namespace xr_ucalib

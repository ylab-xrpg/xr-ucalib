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

#include <xr_ucalib/uc_cam_calib/sfm_calib/handeye_calibrator.h>
// clang-format on

namespace xr_ucalib {

/**
 * @brief Reconstruction geometry storing 3D points and camera poses.
 *
 * This is a simple structure to hold the COLMAP SFM reconstruction results, to
 * facilitate further processing such as scaling and transforming the
 * reconstruction.
 */
class ReconstructionGeometry {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<ReconstructionGeometry>;

  // Factory method to create a shared pointer instance.
  static Ptr Create() { return Ptr(new ReconstructionGeometry()); }

  /**
   * @brief Scale the reconstruction geometry.
   *
   * Includes scaling 3D points and camera translations.
   *
   * @param scale The scale factor.
   */
  void Scale(double scale);

  /**
   * @brief Apply a rigid transformation to the body frame (Right
   * Multiplication).
   *
   * T_W_B2 = T_W_B1 * T_B1_B2, where B1 is the current body frame and B2 is
   * the new body frame.
   *
   * @param T_B1_B2 The transformation from current body to new body frame.
   */
  void TransformBody(const Eigen::Matrix4d& T_B1_B2);

  /**
   * @brief Apply a rigid transformation to the world frame (Left
   * Multiplication).
   *
   * T_W2_B = T_W2_W1 * T_W1_B, where W1 is the current world frame and W2 is
   * the new world frame.
   *
   * @param T_W2_W1 The transformation from current world to new world frame.
   */
  void TransformWorld(const Eigen::Matrix4d& T_W2_W1);

  // Reconstruction camera label.
  std::string label = "";

  // 3D points reconstructed from SFM.
  std::map<int, Eigen::Vector3d> points3d;

  // Camera poses in the reconstruction, transformation from camera to world.
  std::vector<StampedPose> camera_pose_W_C;
};

}  // namespace xr_ucalib

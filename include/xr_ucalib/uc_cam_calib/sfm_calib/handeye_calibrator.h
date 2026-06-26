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

#include <Eigen/Eigen>

namespace xr_ucalib {

/// @brief A timestamped 6-DoF pose.
struct StampedPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp = -1.;

  Eigen::Vector3d trans = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rot_q = Eigen::Quaterniond::Identity();
};

/**
 * @brief Hand-eye calibrator for estimating the extrinsic parameters between
 * body frame and aligning the world frames of two pose sequences.
 *
 * This is a helper class that estimates the extrinsic parameters between camera
 * body frames and aligns them to a common world frame, based on the pose
 * sequences obtained for independent SFM calibration for each camera.
 */
class HandEyeCalibrator {
 public:
  /**
   * @brief Solve for the extrinsic parameters between two camera bodies using
   * hand-eye calibration (AX=XB).
   *
   * Reference: Park, F.C. and Martin, B.J., Robot sensor calibration:
   * solving AX= XB on the Euclidean group, 1994.
   *
   * @param[in] poses_ref Sequence of poses for the reference camera (transform
   * from camera to  world).
   * @param[in] poses_tgt Sequence of poses for the target camera (transform
   * from camera to  world).
   * @param[out] T_ref_tgt Output transformation from target body to reference
   * body.
   * @return true If successful.
   */
  static bool SolveBodyExtrinsics(const std::vector<StampedPose>& poses_ref,
                                  const std::vector<StampedPose>& poses_tgt,
                                  Eigen::Matrix4d& T_ref_tgt);

  /**
   * @brief Align the world frames between two camera pose sequences using
   * Umeyama algorithm.
   *
   * Note that the body frame of the input poses is assumed to be aligned.
   *
   * @param[in] poses_ref Sequence of poses for the reference camera (transform
   * from camera to  world).
   * @param[in] poses_tgt Sequence of poses for the target camera (transform
   * from camera to  world).
   * @param[out] T_ref_tgt Output transformation from target world to reference
   * world.
   * @return true if successful.
   */
  static bool AlignWorldFrames(const std::vector<StampedPose>& poses_ref,
                               const std::vector<StampedPose>& poses_tgt,
                               Eigen::Matrix4d& T_ref_tgt);
};

}  // namespace xr_ucalib

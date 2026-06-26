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

#include "xr_ucalib/uc_cam_calib/sfm_calib/reconstruction_geometry.h"

#include <sophus/se3.hpp>

namespace xr_ucalib {

void ReconstructionGeometry::Scale(double scale) {
  for (auto& kv : points3d) {
    kv.second *= scale;
  }

  for (auto& pose : camera_pose_W_C) {
    pose.trans *= scale;
  }
}

void ReconstructionGeometry::TransformBody(const Eigen::Matrix4d& T_B1_B2) {
  Sophus::SE3d T_body(T_B1_B2);

  for (auto& pose : camera_pose_W_C) {
    Sophus::SE3d T_W_B1(pose.rot_q, pose.trans);
    Sophus::SE3d T_W_B2 = T_W_B1 * T_body;

    pose.trans = T_W_B2.translation();
    pose.rot_q = T_W_B2.unit_quaternion();
  }
}

void ReconstructionGeometry::TransformWorld(const Eigen::Matrix4d& T_W2_W1) {
  Sophus::SE3d T_world(T_W2_W1);

  // Transform 3D points
  for (auto& [id, point] : points3d) {
    point = T_world * point;
  }

  // Transform camera poses
  for (auto& pose : camera_pose_W_C) {
    Sophus::SE3d T_W1_B(pose.rot_q, pose.trans);
    Sophus::SE3d T_W2_B = T_world * T_W1_B;

    pose.trans = T_W2_B.translation();
    pose.rot_q = T_W2_B.unit_quaternion();
  }
}

}  // namespace xr_ucalib

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
#include <string>

#include <nlohmann/json.hpp>
#include <sophus/se3.hpp>

#include "xr_ucalib/uc_common/calib_parameter/cam_radtan_intrinsic.hpp"
#include "xr_ucalib/uc_common/calib_parameter/cam_eqdist_intrinsic.hpp"
#include "xr_ucalib/uc_common/calib_parameter/imu_intrinsic.h"
#include "xr_ucalib/uc_common/calib_parameter/mag_intrinsic.h"
#include "xr_ucalib/uc_common/config/json_adapter.hpp"
#include "xr_ucalib/uc_common/sensor_data/sensor_manager.h"
// clang-format on

namespace xr_ucalib {

/**
 * @brief Class to hold all calibration parameters.
 *
 * The frame conventions used in our system are as follows:
 * - (W): World frame of our system defined by one of the fiducial targets
 * (default is the first target).
 * - (B): Body frame of our system, defined by one of the IMUs
 * (body_frame_label).
 * - (Cb): Base camera frame, defined by one of the cameras (base_camera_label).
 * - (Ci): i-th camera frame.
 * - (Ii): i-th IMU frame.
 * - (Mi): i-th magnetometer frame.
 * - (Ti): i-th fiducial target frame.
 *
 * The extrinsic transformations are represented as follows:
 * - Translations are represented as Sophus::Vector3d, where trans_A_B denotes
 *   the frame B expressed in frame A.
 * - Rotations are represented as Sophus::SO3d, where rot_A_B denotes the
 * rotation from frame B to frame A.
 * - Time offsets are represented as double, where time_offset_A_B denotes the
 * time offset from clock B to clock A, i.e., t_Ii = time_offset_B_Ii + tau_Ii,
 * where tau_Ii is the timestamp in IMU Ii's clock, t_Ii is the corresponding
 * timestamp in the body clock.
 */
class CalibParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CalibParameters>;

  // Factory method to create a shared pointer instance.
  static Ptr Create() { return Ptr(new CalibParameters()); }

  // Set up default values of calibration parameters based on the information in
  // the sensor manager.
  bool SetUpDefaultValues(const SensorManager::Ptr& sensor_manager,
                          bool enable_unified_calib = true);

  // Save calibration parameters to a JSON file (Serialize).
  bool ToJson(const std::string& output_path);

  // Load calibration parameters from a JSON file (Deserialize).
  bool FromJson(const std::string& input_path);

  // Bitmask enum to indicate the status of different calibration parameters.
  enum ParamStatus : uint8_t {
    NONE = 0,                // 0000
    SETUP = 1 << 0,          // 0001 (1)
    SFM_CALIB = 1 << 1,      // 0010 (2)
    CAM_RIG_CALIB = 1 << 2,  // 0100 (4)
    UNIFIED_CALIB = 1 << 3   // 1000 (8)
  };

  uint8_t param_status = ParamStatus::NONE;

  // ==================== General settings ===================
  // Index of the target set as the world frame.
  int world_target_index = -1;
  // Label of the body frame, must be an IMU,
  // only used in "unified calibration" mode.
  std::string body_frame_label = "";
  // Label of the base camera. In unified calibration, we will optimize the
  // extrinsic from the base camera to the body frame, and the extrinsics from
  // other cameras to the base.
  std::string base_camera_label = "";

  // ================= Extrinsic translation =================
  // Base camera frame in body frame.
  Sophus::Vector3d trans_B_Cb = Sophus::Vector3d::Zero();
  // i-th camera frame in base camera frame.
  std::map<std::string, Sophus::Vector3d> trans_Cb_Ci;

  // i-th IMU frame in body frame.
  std::map<std::string, Sophus::Vector3d> trans_B_Ii;

  // i-th Target frame in world frame.
  std::map<int, Sophus::Vector3d> trans_W_Ti;

  // =================== Extrinsic rotation ==================
  // From base camera frame to body frame.
  Sophus::SO3d rot_B_Cb = Sophus::SO3d();
  // From i-th camera frame to base camera frame.
  std::map<std::string, Sophus::SO3d> rot_Cb_Ci;

  // From i-th IMU frame to body frame.
  std::map<std::string, Sophus::SO3d> rot_B_Ii;

  // From i-th magnetometer frame to body frame.
  std::map<std::string, Sophus::SO3d> rot_B_Mi;

  // From i-th target frame to world frame.
  std::map<int, Sophus::SO3d> rot_W_Ti;

  // Gravity direction in world frame (sphere manifold), which represents the
  // rotation between the world frame and the gravity-aligned frame.
  // The gravity magnitude is set by system config.
  Eigen::Vector3d gravity_in_W;

  // Magnetic field direction in world frame (sphere manifold), which represents
  // the rotation between the world frame and the magnetic field-aligned frame.
  // Here we focus on direction only and assume the magnetometer data is
  // normalized to unit length, so the magnitude is not included.
  Eigen::Vector3d mag_in_W;

  // ====================== Time offset ======================
  // From base camera clock to body clock.
  double time_offset_B_Cb = 0.0;
  // From i-th camera clock to base camera clock.
  std::map<std::string, double> time_offset_Cb_Ci;

  // From i-th IMU clock to body clock.
  std::map<std::string, double> time_offset_B_Ii;

  // From i-th magnetometer clock to body clock.
  std::map<std::string, double> time_offset_B_Mi;

  // ======================= Intrinsic =======================
  // Camera intrinsic parameters for each camera.
  std::map<std::string, CamIntrinsicBase::Ptr> cam_intrinsics;

  // IMU intrinsic parameters for each IMU.
  std::map<std::string, ImuIntrinsic::Ptr> imu_intrinsics;

  // Magnetometer intrinsic parameters for each magnetometer.
  // TODO: Now, Magnetometer data is assumed to be pre-calibrated via ellipsoid
  // fitting and normalized to unit length, so we don't include intrinsic
  // parameters for magnetometers. We can add them in the future if needed.
  // std::map<std::string, MagIntrinsic::Ptr> mag_intrinsics;

 private:
  CalibParameters() = default;
};

// JSON (de)serialization support for basic fields of CalibParameters.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(
    CalibParameters, world_target_index, body_frame_label, base_camera_label,
    trans_B_Cb, trans_Cb_Ci, trans_B_Ii, trans_W_Ti, rot_B_Cb, rot_Cb_Ci,
    rot_B_Ii, rot_B_Mi, rot_W_Ti, gravity_in_W, mag_in_W, time_offset_B_Cb,
    time_offset_Cb_Ci, time_offset_B_Ii, time_offset_B_Mi);

}  // namespace xr_ucalib
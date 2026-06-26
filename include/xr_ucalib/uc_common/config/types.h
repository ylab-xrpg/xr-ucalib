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

#include "xr_ucalib/uc_common/utils/enum_utils.h"

namespace xr_ucalib {

/// @brief Enumeration for sensor types.
MAKE_ENUM_CLASS(SensorType, -1, INVALID, CAMERA, IMU, MAGNETOMETER, FIDUCIAL)

/// @brief Enumeration for camera model types. Now only supports Kalibr RADTAN
/// and Equidistant models.
MAKE_ENUM_CLASS(CamModelType, -1, INVALID, RADTAN, EQUIDISTANT)

/// @brief Enumeration for IMU model types.
MAKE_ENUM_CLASS(ImuModelType, -1, INVALID, CALIBRATED, SCALE, MISALIGN,
                SCALE_MISALIGN)

/// @brief Enumeration for fiducial marker types. Now only supports AprilTag.
MAKE_ENUM_CLASS(FiducialType, -1, INVALID, APRILTAG)

/// @brief Enumeration for detection display modes.
MAKE_ENUM_CLASS(DetectionDisplayMode, -1, NONE, STEP, CONTINUOUS)

}  // namespace xr_ucalib

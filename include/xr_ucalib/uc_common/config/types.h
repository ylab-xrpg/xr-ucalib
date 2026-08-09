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

#include <vector>

#include "xr_ucalib/uc_common/utils/enum_utils.h"

namespace xr_ucalib {

/// @brief Enumeration for sensor types.
MAKE_ENUM_CLASS(SensorType, -1, INVALID, CAMERA, IMU, MAGNETOMETER, FIDUCIAL)

/// @brief Enumeration for supported camera model types.
MAKE_ENUM_CLASS(CamModelType, -1, INVALID, RADTAN, EQUIDISTANT,
                RAD_TAN_THIN_PRISM_FISHEYE, RAD_TAN_THIN_PRISM_FISHEYE_620)

/// @brief Check whether a camera model is a Fisheye624 family member.
inline bool IsFisheye624Variant(CamModelType type) {
  return type == CamModelType::RAD_TAN_THIN_PRISM_FISHEYE ||
         type == CamModelType::RAD_TAN_THIN_PRISM_FISHEYE_620;
}

/// @brief Return parameter indices that are disabled for a Fisheye624 variant.
inline std::vector<int> GetFisheye624ConstantParams(CamModelType type) {
  if (type == CamModelType::RAD_TAN_THIN_PRISM_FISHEYE_620) {
    // Fisheye620 disables the four thin-prism coefficients s0-s3.
    return {12, 13, 14, 15};
  }
  return {};
}

/**
 * @brief Set disabled Fisheye624 parameters to zero before optimization.
 *
 * A Ceres SubsetManifold preserves the current parameter values. Explicitly
 * normalizing disabled terms prevents a Fisheye620 prior or serialized result
 * from retaining non-zero thin-prism coefficients.
 */
inline bool ZeroFisheye624ConstantParams(CamModelType type,
                                         std::vector<double>* parameters) {
  const auto constant_params = GetFisheye624ConstantParams(type);
  if (constant_params.empty()) {
    return true;
  }
  constexpr size_t kFisheye624ParameterSize = 16;
  if (parameters == nullptr || parameters->size() != kFisheye624ParameterSize) {
    return false;
  }
  for (const int index : constant_params) {
    parameters->at(index) = 0.0;
  }
  return true;
}

/// @brief Enumeration for IMU model types.
MAKE_ENUM_CLASS(ImuModelType, -1, INVALID, CALIBRATED, SCALE, MISALIGN,
                SCALE_MISALIGN)

/// @brief Enumeration for fiducial marker types. Now only supports AprilTag.
MAKE_ENUM_CLASS(FiducialType, -1, INVALID, APRILTAG)

/// @brief Enumeration for detection display modes.
MAKE_ENUM_CLASS(DetectionDisplayMode, -1, NONE, STEP, CONTINUOUS)

}  // namespace xr_ucalib

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

#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "xr_ucalib/uc_common/config/types.h"

namespace xr_ucalib {

/// @brief Base class for camera intrinsic parameters.
class CamIntrinsicBase {
 public:
  using Ptr = std::shared_ptr<CamIntrinsicBase>;

  virtual ~CamIntrinsicBase() = default;

  // Camera model type.
  CamModelType cam_model_type = CamModelType::INVALID;

  inline std::string IntrinsicsToString(int precision = 6) const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    oss << "[";

    for (size_t i = 0; i < parameters.size(); ++i) {
      oss << parameters[i];
      if (i + 1 < parameters.size()) {
        oss << ", ";
      }
    }

    oss << "]";
    return oss.str();
  }

  // Initial focal length for optimization initialization.
  int initial_focal_length = 0;

  // Image dimensions.
  int width = 0;
  int height = 0;

  // COLMAP camera ID used in SFM calibration.
  unsigned int colmap_cam_id = 0;

  // Number of intrinsic parameters.
  int parameter_size = 0;

  // Intrinsic parameters, including fx fy cx cy and distortion parameters.
  std::vector<double> parameters;
};

}  // namespace xr_ucalib

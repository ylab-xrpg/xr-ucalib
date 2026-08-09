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

#include <opencv2/core/types.hpp>
#include <vector>

#include "xr_ucalib/uc_common/calib_parameter/cam_intrinsic_base.h"

namespace xr_ucalib {

/**
 * @brief Convert distorted pixels to normalized pinhole coordinates for PnP.
 *
 * The output size matches the input size. Points that cannot be inverted are
 * represented by (NaN, NaN), allowing callers to filter them together with the
 * corresponding 3D observations.
 *
 * @return false for an invalid intrinsic vector, unsupported model, or null
 * output pointer.
 */
bool UndistortCameraPoints(const CamIntrinsicBase::Ptr& cam_intrinsic,
                           const std::vector<cv::Point2d>& image_points,
                           std::vector<cv::Point2d>* normalized_points);

}  // namespace xr_ucalib

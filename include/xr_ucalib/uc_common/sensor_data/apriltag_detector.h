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
#include <memory>

#include <apriltags/Tag36h11.h>
#include <apriltags/TagDetector.h>

#include "xr_ucalib/uc_common/sensor_data/fiducial_detector_base.h"
// clang-format on

namespace xr_ucalib {

/// @brief AprilTag fiducial marker detector implementation.
class AprilTagDetecter : public FiducialDetecterBase {
 public:
  using Ptr = std::shared_ptr<AprilTagDetecter>;

  // Factory method to create a shared pointer instance.
  static Ptr Create() { return Ptr(new AprilTagDetecter()); }

 private:
  AprilTagDetecter();

  // Override DetectFrame to detect 2D corners of AprilTag fiducials in a single
  // frame.
  bool DetectFrame(const cv::Mat &image, CamFrame::Ptr &cam_frame) override;

  // AprilTag family (36h11).
  AprilTags::TagCodes tag_code_;
  // AprilTag detector instance.
  std::unique_ptr<AprilTags::TagDetector> tag_detector_;
};

}  // namespace xr_ucalib

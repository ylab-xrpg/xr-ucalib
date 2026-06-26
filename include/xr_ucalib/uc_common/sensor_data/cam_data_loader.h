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

#include <memory>
#include <string>

#include "xr_ucalib/uc_common/config/types.h"
#include "xr_ucalib/uc_common/sensor_data/sensor_data_types.h"

namespace xr_ucalib {

/// @brief Camera data loader class.
class CamDataLoader : public DataLoaderBase {
 public:
  using Ptr = std::shared_ptr<CamDataLoader>;

  // Factory method to create a shared pointer instance.
  static Ptr Create() { return Ptr(new CamDataLoader()); }

  // Override Load method to load camera data from file.
  bool Load(const std::string& data_path) override;

  // Get the loaded camera sequence.
  CamSequence::Ptr GetSequence() const { return sequence_; };

  // Set number of threads for detection.
  void SetNumThreads(int num_threads) { num_threads_ = num_threads; }

  // Set the fiducial marker type for detection.
  void SetFiducialType(const FiducialType& fiducial_type) {
    fiducial_type_ = fiducial_type;
  }

  // Get image dimensions.
  int GetWidth() const { return img_width_; }
  int GetHeight() const { return img_height_; }

  // Show the detected keypoints in images. (Reloading the image is required to
  // visualize detections, so this function is recommended only for debugging.)
  bool ShowDetections(bool step_mode = false);

  // Save the detected keypoints to a JSON file (Serialize).
  bool SaveDetections(const std::string& output_path);

  // Read the detected keypoints from a JSON file (Deserialize).
  bool ReadDetections(const std::string& input_path);

 private:
  CamDataLoader() = default;

  CamSequence::Ptr sequence_;

  // Number of threads for detecting fiducials in images.
  int num_threads_ = 8;

  int img_width_ = 0;
  int img_height_ = 0;

  FiducialType fiducial_type_ = FiducialType::INVALID;
};

}  // namespace xr_ucalib
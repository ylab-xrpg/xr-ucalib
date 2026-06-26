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
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

#include "xr_ucalib/uc_common/sensor_data/sensor_data_types.h"
// clang-format on

namespace xr_ucalib {

/// @brief Base class for fiducial marker detection in camera images.
class FiducialDetecterBase {
 public:
  using Ptr = std::shared_ptr<FiducialDetecterBase>;

  // Virtual destructor to ensure proper cleanup of derived classes.
  virtual ~FiducialDetecterBase() = default;

  /**
   * @brief Process a sequence of images from directory and detect fiducial
   * markers.
   *
   * @param[in] seq_dir Directory containing image sequence.
   * @param[out] cam_seq Pointer to store the resulting camera sequence with
   * detections.
   * @param[out] width Width of the images in the sequence.
   * @param[out] height Height of the images in the sequence.
   * @return true if detection is successful, false otherwise.
   */
  bool DetectSequence(const std::string &seq_dir, CamSequence::Ptr &cam_seq,
                      int &width, int &height);

  /**
   * @brief Multi-threaded version of DetectSequence for improved performance.
   *
   * @param[in] seq_dir Directory containing image sequence.
   * @param[out] cam_seq Pointer to store the resulting camera sequence with
   * detections.
   * @param[out] width Width of the images in the sequence.
   * @param[out] height Height of the images in the sequence.
   * @param num_threads Number of threads to use for parallel processing.
   * @return true if detection is successful, false otherwise.
   */
  bool DetectSequenceMT(const std::string &seq_dir, CamSequence::Ptr &cam_seq,
                        int &width, int &height, const int num_threads = 8);

 protected:
  /**
   * @brief Pure virtual method to detect fiducial markers in a single image
   * frame, focusing on extracting the 2D corner coordinates.
   *
   * Fiducial structure (corner order) on the target board is detailed in the
   * TargetCorner3D class
   *
   * @param[in] image Input image in which to detect fiducial markers.
   * @param[out] cam_frame Output camera frame with detected keypoints.
   * @return true if detection is successful, false otherwise.
   */
  virtual bool DetectFrame(const cv::Mat &image, CamFrame::Ptr &cam_frame) = 0;

  /**
   * @brief Extract timestamp from image file path.
   *
   * @param[in] img_path Image file path.
   * @param[out] timestamp Extracted timestamp in seconds.
   * @return true if timestamp extraction is successful, false otherwise.
   */
  bool Path2Time(std::string img_path, double &timestamp);

  // Valid image file extensions.
  static inline const std::vector<std::string> kValidImageExtensions = {
      ".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif"};
};

}  // namespace xr_ucalib
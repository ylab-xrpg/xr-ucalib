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

#include "xr_ucalib/uc_common/sensor_data/apriltag_detector.h"

#include <vector>

namespace xr_ucalib {

AprilTagDetecter::AprilTagDetecter() : tag_code_(AprilTags::tagCodes36h11) {
  tag_detector_ = std::make_unique<AprilTags::TagDetector>(tag_code_);
}

bool AprilTagDetecter::DetectFrame(const cv::Mat &image,
                                   CamFrame::Ptr &cam_frame) {
  int img_width = image.cols;
  int img_height = image.rows;

  // Step 1: Detect AprilTags in the image.
  std::vector<AprilTags::TagDetection> detections =
      tag_detector_->extractTags(image);
  if (detections.empty()) {
    spdlog::debug("No AprilTag detected in the current frame.");
    return false;
  }

  // ===========================================================================

  // Step 2: Calculate average tag edge length in pixels
  double total_edge_length = 0.0;
  int edge_count = 0;

  for (const auto &detection : detections) {
    // Calculate edge lengths for each tag (4 edges per tag)
    for (int i = 0; i < 4; ++i) {
      int next_i = (i + 1) % 4;  // Wrap around: 0->1, 1->2, 2->3, 3->0

      double dx = detection.p[next_i].first - detection.p[i].first;
      double dy = detection.p[next_i].second - detection.p[i].second;
      double edge_length = std::sqrt(dx * dx + dy * dy);

      total_edge_length += edge_length;
      edge_count++;
    }
  }
  double avg_edge_length = total_edge_length / edge_count;

  // Calculate reference parameter: 1/10 of edge length, rounded up, clamped to
  // [3, 10]
  int edge_len_tenth = static_cast<int>(std::ceil(avg_edge_length / 10.0));
  edge_len_tenth = std::max(3, std::min(10, edge_len_tenth));

  // ===========================================================================

  // Step 3: Filter detections based on border distance and quality.
  const int kMinBorderDistance = edge_len_tenth;
  auto iter = detections.begin();
  while (iter != detections.end()) {
    bool remove_flag = false;

    // Check if any corner is too close to the border
    for (int i = 0; i < 4; ++i) {
      float x = iter->p[i].first;
      float y = iter->p[i].second;
      if (x < kMinBorderDistance || x > img_width - kMinBorderDistance ||
          y < kMinBorderDistance || y > img_height - kMinBorderDistance) {
        remove_flag = true;
        break;
      }
    }

    // Check if detection quality is good
    if (!iter->good) {
      remove_flag = true;
    }

    // Remove bad detections, erase() returns iterator to next element
    if (remove_flag) {
      iter = detections.erase(iter);
    } else {
      ++iter;
    }
  }

  if (detections.empty()) {
    spdlog::debug("No AprilTag detected in the current frame.");
    return false;
  }

  // ===========================================================================

  // Step 4: Refine corner locations to subpixel accuracy.
  const int kSubPixWinSize =
      (edge_len_tenth % 2 == 0) ? (edge_len_tenth + 1) : edge_len_tenth;
  constexpr int kSubPixMaxIter = 30;
  constexpr double kSubPixEps = 0.01;
  cv::Mat corner_src(4 * detections.size(), 2, CV_32F);
  for (size_t i = 0; i < detections.size(); ++i) {
    for (int j = 0; j < 4; ++j) {
      // Explicit cast: double -> float (AprilTag detection uses double
      // internally)
      corner_src.at<float>(4 * i + j, 0) =
          static_cast<float>(detections[i].p[j].first);
      corner_src.at<float>(4 * i + j, 1) =
          static_cast<float>(detections[i].p[j].second);
    }
  }
  cv::Mat corner_refined = corner_src.clone();
  cv::cornerSubPix(image, corner_refined,
                   cv::Size(kSubPixWinSize, kSubPixWinSize), cv::Size(-1, -1),
                   cv::TermCriteria(cv::TermCriteria::Type::MAX_ITER +
                                        cv::TermCriteria::Type::EPS,
                                    kSubPixMaxIter, kSubPixEps));

  // ===========================================================================

  // Step 5: Populate cam_frame with refined corners.
  const double kHalfEdgeTenth = edge_len_tenth / 2.0;
  const double kMaxDispSquared = kHalfEdgeTenth * kHalfEdgeTenth;
  for (size_t i = 0; i < detections.size(); ++i) {
    int tag_id = detections[i].id;

    /**
     * Reorder AprilTag corner IDs to OpenCV convention.
     *
     * 3--------2       0--------1
     * |        |       |        |
     * |  ATag  |  -->  |  PUSH  |
     * |        |       |        |
     * 0--------1       3--------2
     *
     */
    int opencv_order[4] = {3, 2, 1, 0};

    // add four points per tag
    for (int j = 0; j < 4; j++) {
      // Explicit cast: float -> double (refined corners for high-precision
      // calibration)
      double corner_x =
          static_cast<double>(corner_refined.row(4 * i + j).at<float>(0));
      double corner_y =
          static_cast<double>(corner_refined.row(4 * i + j).at<float>(1));

      double cornerRaw_x =
          static_cast<double>(corner_src.row(4 * i + j).at<float>(0));
      double cornerRaw_y =
          static_cast<double>(corner_src.row(4 * i + j).at<float>(1));

      double pix_displacement_squared =
          (corner_x - cornerRaw_x) * (corner_x - cornerRaw_x) +
          (corner_y - cornerRaw_y) * (corner_y - cornerRaw_y);

      // Only add point if the displacement in the subpixel refinement is below
      // the threshold.
      if (pix_displacement_squared <= kMaxDispSquared) {
        int corner_id = tag_id * 4 + opencv_order[j];

        cam_frame->keypoints.emplace(corner_id,
                                     Eigen::Vector2d(corner_x, corner_y));
      }
    }
  }

  // ===========================================================================

  return true;
}

}  // namespace xr_ucalib
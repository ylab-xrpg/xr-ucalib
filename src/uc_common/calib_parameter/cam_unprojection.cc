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

#include "xr_ucalib/uc_common/calib_parameter/cam_unprojection.h"

#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "xr_ucalib/uc_common/calib_parameter/cam_radtan_thin_prism_fisheye_intrinsic.hpp"

namespace xr_ucalib {
namespace {

size_t ExpectedParameterSize(CamModelType model_type) {
  if (model_type == CamModelType::RADTAN ||
      model_type == CamModelType::EQUIDISTANT) {
    return 8;
  }
  if (IsFisheye624Variant(model_type)) {
    return 16;
  }
  return 0;
}

}  // namespace

bool UndistortCameraPoints(const CamIntrinsicBase::Ptr& cam_intrinsic,
                           const std::vector<cv::Point2d>& image_points,
                           std::vector<cv::Point2d>* normalized_points) {
  if (cam_intrinsic == nullptr || normalized_points == nullptr) {
    return false;
  }
  const size_t expected_size =
      ExpectedParameterSize(cam_intrinsic->cam_model_type);
  if (expected_size == 0 ||
      cam_intrinsic->parameter_size != static_cast<int>(expected_size) ||
      cam_intrinsic->parameters.size() != expected_size) {
    return false;
  }

  const auto& parameters = cam_intrinsic->parameters;
  if (IsFisheye624Variant(cam_intrinsic->cam_model_type)) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    normalized_points->clear();
    normalized_points->reserve(image_points.size());
    for (const cv::Point2d& image_point : image_points) {
      Eigen::Vector2d normalized_point;
      if (CamRadTanThinPrismFisheyeIntrinsic::Image2Cam(
              Eigen::Vector2d(image_point.x, image_point.y), parameters.data(),
              normalized_point)) {
        normalized_points->emplace_back(normalized_point.x(),
                                        normalized_point.y());
      } else {
        normalized_points->emplace_back(nan, nan);
      }
    }
    return true;
  }

  cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  camera_matrix.at<double>(0, 0) = parameters[0];
  camera_matrix.at<double>(1, 1) = parameters[1];
  camera_matrix.at<double>(0, 2) = parameters[2];
  camera_matrix.at<double>(1, 2) = parameters[3];
  const cv::Mat distortion = (cv::Mat_<double>(4, 1) << parameters[4],
                              parameters[5], parameters[6], parameters[7]);

  if (cam_intrinsic->cam_model_type == CamModelType::RADTAN) {
    cv::undistortPoints(image_points, *normalized_points, camera_matrix,
                        distortion);
  } else {
    cv::fisheye::undistortPoints(image_points, *normalized_points,
                                 camera_matrix, distortion);
  }
  return true;
}

}  // namespace xr_ucalib

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

// clang-format off
#include "xr_ucalib/uc_cam_calib/cam_rig_calib/reproj_evaluator.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <tuple>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

#include "xr_ucalib/uc_cam_calib/cam_rig_calib/cam_reproj_cost.hpp"
#include "xr_ucalib/uc_common/calib_parameter/cam_eqdist_intrinsic.hpp"
#include "xr_ucalib/uc_common/calib_parameter/cam_radtan_intrinsic.hpp"
// clang-format on

namespace xr_ucalib {

namespace {

/// @brief Scalar statistics of reprojection error magnitudes.
struct ErrorStats {
  size_t count = 0;
  double mean = 0.0;
  double median = 0.0;
  double min = 0.0;
  double max = 0.0;
};

/// @brief Compute mean/median/min/max statistics from 2D residual vectors.
ErrorStats ComputeErrorStats(const std::vector<cv::Point2d>& errors_2d) {
  ErrorStats stats;
  if (errors_2d.empty()) {
    return stats;
  }

  // Convert 2D residuals to magnitudes.
  std::vector<double> mags;
  mags.reserve(errors_2d.size());
  double sum = 0.0;
  for (const auto& e : errors_2d) {
    const double dist = std::hypot(e.x, e.y);
    mags.push_back(dist);
    sum += dist;
  }

  // Compute scalar statistics.
  std::sort(mags.begin(), mags.end());
  stats.count = mags.size();
  stats.mean = sum / static_cast<double>(mags.size());
  stats.min = mags.front();
  stats.max = mags.back();
  if (mags.size() % 2 == 1) {
    stats.median = mags[mags.size() / 2];
  } else {
    const size_t mid = mags.size() / 2;
    stats.median = 0.5 * (mags[mid - 1] + mags[mid]);
  }

  return stats;
}

/// @brief Compute mean and covariance of 2D residual vectors.
std::tuple<cv::Point2d, double, double, double> Compute2DStats(
    const std::vector<cv::Point2d>& errors_2d) {
  if (errors_2d.empty()) {
    return {cv::Point2d(0.0, 0.0), 0.0, 0.0, 0.0};
  }

  // Compute 2D mean.
  cv::Point2d mean(0.0, 0.0);
  for (const auto& e : errors_2d) {
    mean.x += e.x;
    mean.y += e.y;
  }
  mean.x /= errors_2d.size();
  mean.y /= errors_2d.size();

  // Compute 2D covariance entries.
  double cov_xx = 0.0, cov_yy = 0.0, cov_xy = 0.0;
  for (const auto& e : errors_2d) {
    double dx = e.x - mean.x;
    double dy = e.y - mean.y;
    cov_xx += dx * dx;
    cov_yy += dy * dy;
    cov_xy += dx * dy;
  }
  cov_xx /= errors_2d.size();
  cov_yy /= errors_2d.size();
  cov_xy /= errors_2d.size();

  return {mean, cov_xx, cov_yy, cov_xy};
}

/// @brief Save reprojection residual vectors and summary statistics to txt.
bool SaveErrorDataTxt(const std::string& txt_file_path,
                      const std::string& title_label, int num_frames,
                      const ErrorStats& stats,
                      const std::vector<cv::Point2d>& errors_2d,
                      bool plain_stat_lines) {
  std::ofstream txt_file(txt_file_path);
  if (!txt_file.is_open()) {
    return false;
  }

  // Write header and summary statistics.
  txt_file << "# Reprojection Error Data for Camera: " << title_label << "\n";
  txt_file << "# Number of frames: " << num_frames << "\n";
  txt_file << "# Number of points: " << errors_2d.size() << "\n";
  txt_file << "# Error Magnitude Statistics:\n";

  const std::string prefix = plain_stat_lines ? "" : "# ";
  txt_file << prefix << "Mean:   " << std::fixed << std::setprecision(6)
           << stats.mean << " px\n";
  txt_file << prefix << "Median: " << std::fixed << std::setprecision(6)
           << stats.median << " px\n";
  txt_file << prefix << "Min:    " << std::fixed << std::setprecision(6)
           << stats.min << " px\n";
  txt_file << prefix << "Max:    " << std::fixed << std::setprecision(6)
           << stats.max << " px\n";
  txt_file << "# Format: error_x error_y (in pixels)\n";
  txt_file << "#\n";

  // Write residual vectors.
  for (const auto& e : errors_2d) {
    txt_file << std::fixed << std::setprecision(6) << e.x << " " << e.y << "\n";
  }

  return true;
}

/// @brief Draw reprojection error distribution and save as png.
bool DrawErrorPlot(const std::string& cam_label,
                   const std::vector<cv::Point2d>& errors_2d,
                   const ErrorStats& stats, int num_frames,
                   const std::string& file_path) {
  if (errors_2d.empty()) {
    return false;
  }

  // Step 1: Prepare canvas and plotting scale.
  const int img_size = 800;
  const int margin = 90;
  const int plot_size = img_size - 2 * margin;
  const cv::Point2i center(img_size / 2, img_size / 2);

  auto [mean_2d, cov_xx, cov_yy, cov_xy] = Compute2DStats(errors_2d);

  double trace = cov_xx + cov_yy;
  double det = cov_xx * cov_yy - cov_xy * cov_xy;
  double discriminant = trace * trace / 4.0 - det;
  double sigma_3_max = 0.0;
  if (discriminant >= 0) {
    double sqrt_disc = std::sqrt(discriminant);
    double lambda1 = std::max(trace / 2.0 + sqrt_disc, 1e-6);
    sigma_3_max = 3.0 * std::sqrt(lambda1);
  }

  double max_range = std::max({stats.max * 1.2, sigma_3_max * 1.2, 1.0});
  max_range = std::ceil(max_range);
  double scale = (plot_size / 2.0) / max_range;

  // ==========================================================================

  // Step 2: Draw grid, axes, and ticks.
  cv::Mat img(img_size, img_size, CV_8UC3, cv::Scalar(255, 255, 255));

  cv::Scalar grid_color(220, 220, 220);
  for (int i = -4; i <= 4; ++i) {
    int offset = static_cast<int>(i * (plot_size / 8.0));
    cv::line(img, cv::Point(center.x + offset, margin),
             cv::Point(center.x + offset, img_size - margin), grid_color, 1);
    cv::line(img, cv::Point(margin, center.y + offset),
             cv::Point(img_size - margin, center.y + offset), grid_color, 1);
  }

  cv::Scalar axis_color(100, 100, 100);
  cv::line(img, cv::Point(margin, center.y),
           cv::Point(img_size - margin, center.y), axis_color, 2);
  cv::line(img, cv::Point(center.x, margin),
           cv::Point(center.x, img_size - margin), axis_color, 2);
  cv::putText(img, "Error X (px)",
              cv::Point(img_size - margin - 90, center.y + 45),
              cv::FONT_HERSHEY_SIMPLEX, 0.55, axis_color, 1.5);
  cv::putText(img, "Error Y (px)", cv::Point(center.x - 110, margin + 6),
              cv::FONT_HERSHEY_SIMPLEX, 0.55, axis_color, 1.5);

  for (int i = -2; i <= 2; ++i) {
    if (i == 0) continue;
    int tick_val = static_cast<int>(i * (max_range / 2.0));
    int offset = static_cast<int>(i * (plot_size / 4.0));
    std::string tick_str = cv::format("%d", tick_val);
    cv::putText(img, tick_str, cv::Point(center.x + offset - 10, center.y + 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, axis_color, 1.5);
    cv::putText(img, tick_str, cv::Point(center.x + 8, center.y - offset + 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, axis_color, 1.5);
  }

  // ==========================================================================

  // Step 3: Draw residual points and 3-sigma ellipse.
  cv::Scalar point_color(255, 0, 0);
  const int cross_size = 3;
  for (const auto& e : errors_2d) {
    int px = center.x + static_cast<int>(e.x * scale);
    int py = center.y - static_cast<int>(e.y * scale);
    if (px < margin || px > img_size - margin || py < margin ||
        py > img_size - margin) {
      continue;
    }
    cv::line(img, cv::Point(px - cross_size, py - cross_size),
             cv::Point(px + cross_size, py + cross_size), point_color, 1.2);
    cv::line(img, cv::Point(px - cross_size, py + cross_size),
             cv::Point(px + cross_size, py - cross_size), point_color, 1.2);
  }

  cv::Point2i mean_pt(center.x + static_cast<int>(mean_2d.x * scale),
                      center.y - static_cast<int>(mean_2d.y * scale));
  cv::circle(img, mean_pt, 5, cv::Scalar(0, 255, 0), -1);

  if (discriminant >= 0) {
    double sqrt_disc = std::sqrt(discriminant);
    double lambda1 = std::max(trace / 2.0 + sqrt_disc, 1e-6);
    double lambda2 = std::max(trace / 2.0 - sqrt_disc, 1e-6);
    double angle = 0.0;
    if (std::abs(cov_xy) > 1e-10) {
      angle = 0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);
    }
    double axis_a = 3.0 * std::sqrt(lambda1) * scale;
    double axis_b = 3.0 * std::sqrt(lambda2) * scale;
    cv::Point2i ellipse_center(center.x + static_cast<int>(mean_2d.x * scale),
                               center.y - static_cast<int>(mean_2d.y * scale));
    cv::ellipse(img, ellipse_center,
                cv::Size(static_cast<int>(axis_a), static_cast<int>(axis_b)),
                -angle * 180.0 / M_PI, 0, 360, cv::Scalar(0, 0, 255), 2);
  }

  // ==========================================================================

  // Step 4: Draw textual statistics and legend.
  int text_y = 25;
  cv::putText(img,
              cv::format("Camera: %s, from %d frames, %zu points",
                         cam_label.c_str(), num_frames, errors_2d.size()),
              cv::Point(10, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.6,
              cv::Scalar(0, 0, 0), 2);
  text_y += 25;
  cv::putText(img, "Error Magnitude Statistics:", cv::Point(10, text_y),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1.5);
  text_y += 20;
  cv::putText(img,
              cv::format("mean=%.2fpx, median=%.2fpx, min=%.2fpx, max=%.2fpx",
                         stats.mean, stats.median, stats.min, stats.max),
              cv::Point(10, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(0, 0, 0), 1.5);

  int legend_y = img_size - 55;
  cv::putText(img, "Vector-based Statistics:", cv::Point(10, legend_y),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1.5);
  legend_y += 20;
  const int lx = 20;
  const int ly = legend_y;
  const int lcross = 6;
  cv::line(img, cv::Point(lx - lcross, ly - lcross),
           cv::Point(lx + lcross, ly + lcross), point_color, 2);
  cv::line(img, cv::Point(lx - lcross, ly + lcross),
           cv::Point(lx + lcross, ly - lcross), point_color, 2);
  cv::putText(img, "Error points", cv::Point(35, legend_y + 5),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1.5);
  cv::circle(img, cv::Point(150, legend_y), 4, cv::Scalar(0, 255, 0), -1);
  cv::putText(img, "Mean (vector)", cv::Point(160, legend_y + 5),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1.5);
  cv::ellipse(img, cv::Point(300, legend_y), cv::Size(15, 10), 0, 0, 360,
              cv::Scalar(0, 0, 255), 2);
  cv::putText(img, "3-sigma ellipse", cv::Point(320, legend_y + 5),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1.5);

  // ==========================================================================

  // Save figure.
  return cv::imwrite(file_path, img);
}

/// @brief Uniformly select frame indices for visualization sampling.
std::vector<size_t> SelectUniformIndices(size_t total_count,
                                         size_t target_count) {
  std::vector<size_t> indices;
  if (total_count == 0 || target_count == 0) {
    return indices;
  }

  // Keep all indices when target count is large enough.
  if (target_count >= total_count) {
    indices.resize(total_count);
    for (size_t i = 0; i < total_count; ++i) {
      indices[i] = i;
    }
    return indices;
  }

  // Uniformly sample index positions.
  indices.reserve(target_count);
  for (size_t i = 0; i < target_count; ++i) {
    size_t idx = (i * (total_count - 1)) / (target_count - 1);
    indices.push_back(idx);
  }
  return indices;
}

}  // namespace

bool ReprojEvaluator::ComputeReprojError(
    const CamPoses& cam_poses_W_Cb,
    const FrameCorrespondence& frame_correspondence,
    ReprojErrorPerFrame& reproj_errors) {
  reproj_errors.clear();

  if (cam_poses_W_Cb.empty() || frame_correspondence.empty()) {
    spdlog::error(
        "Empty camera poses or frame correspondence for reprojection error "
        "computation.");
    return false;
  }

  const auto& target_corners = sensor_manager_->GetTargetCorners()->corners;

  //  Iterate over frame correspondences with valid base-camera pose.
  // Iterate through all frame correspondences.
  for (const auto& [frame_idx, cam_frames_map] : frame_correspondence) {
    auto pose_it = cam_poses_W_Cb.find(frame_idx);
    if (pose_it == cam_poses_W_Cb.end()) continue;

    const Eigen::Vector3d& trans_W_Cb = pose_it->second.first;
    const Eigen::Quaterniond& rot_W_Cb = pose_it->second.second;

    // Evaluate all camera/keypoint residuals in this frame.
    // Iterate through all camera frames in the correspondence.
    for (const auto& [cam_label, frame_ptr] : cam_frames_map) {
      if (frame_ptr->keypoints.empty()) continue;

      const auto& trans_Cb_Ci = calib_parameters_->trans_Cb_Ci.at(cam_label);
      const auto& rot_Cb_Ci = calib_parameters_->rot_Cb_Ci.at(cam_label);
      const auto& cam_intrinsic =
          calib_parameters_->cam_intrinsics.at(cam_label);

      for (const auto& [kp_id, observed_kp] : frame_ptr->keypoints) {
        if (target_corners.count(kp_id) == 0) continue;

        const auto& target_corner = target_corners.at(kp_id);
        int target_idx = target_corner.target_idx;
        if (calib_parameters_->trans_W_Ti.count(target_idx) == 0) continue;

        const auto& trans_W_Ti = calib_parameters_->trans_W_Ti.at(target_idx);
        const auto& rot_W_Ti = calib_parameters_->rot_W_Ti.at(target_idx);

        // Use the CamReprojCost to compute residuals
        CamReprojCost cost_functor(cam_intrinsic, target_corner.position_local,
                                   observed_kp);
        double residuals[2];
        cost_functor(trans_W_Cb.data(), rot_W_Cb.coeffs().data(),
                     trans_Cb_Ci.data(),
                     rot_Cb_Ci.unit_quaternion().coeffs().data(),
                     cam_intrinsic->parameters.data(), trans_W_Ti.data(),
                     rot_W_Ti.unit_quaternion().coeffs().data(), residuals);

        reproj_errors[cam_label][frame_ptr].emplace_back(residuals[0],
                                                         residuals[1]);
      }
    }
  }

  return true;
}

size_t ReprojEvaluator::RemoveReprojOutliers(
    const CamPoses& cam_poses_W_Cb,
    const FrameCorrespondence& frame_correspondence) {
  const auto& cam_configs = sensor_manager_->GetAllCamConfigs();
  const auto& target_corners = sensor_manager_->GetTargetCorners()->corners;
  size_t total_removed = 0;

  // Iterate over frame correspondences with valid base-camera pose.
  for (const auto& [frame_idx, cam_frames_map] : frame_correspondence) {
    auto pose_it = cam_poses_W_Cb.find(frame_idx);
    if (pose_it == cam_poses_W_Cb.end()) continue;

    const Eigen::Vector3d& trans_W_Cb = pose_it->second.first;
    const Eigen::Quaterniond& rot_W_Cb = pose_it->second.second;

    // Detect and remove outlier keypoints for each camera frame.
    for (const auto& [cam_label, frame_ptr] : cam_frames_map) {
      if (frame_ptr->keypoints.empty()) continue;

      // Check if outlier rejection is enabled for this camera.
      auto config_it = cam_configs.find(cam_label);
      if (config_it == cam_configs.end()) continue;
      double threshold = config_it->second.reproj_threshold;
      if (threshold < 0) continue;

      const auto& trans_Cb_Ci = calib_parameters_->trans_Cb_Ci.at(cam_label);
      const auto& rot_Cb_Ci = calib_parameters_->rot_Cb_Ci.at(cam_label);
      const auto& cam_intrinsic =
          calib_parameters_->cam_intrinsics.at(cam_label);

      // Collect landmark IDs of outlier keypoints.
      std::vector<int> outlier_ids;
      for (const auto& [kp_id, observed_kp] : frame_ptr->keypoints) {
        if (target_corners.count(kp_id) == 0) continue;

        const auto& target_corner = target_corners.at(kp_id);
        int target_idx = target_corner.target_idx;
        if (calib_parameters_->trans_W_Ti.count(target_idx) == 0) continue;

        const auto& trans_W_Ti = calib_parameters_->trans_W_Ti.at(target_idx);
        const auto& rot_W_Ti = calib_parameters_->rot_W_Ti.at(target_idx);

        CamReprojCost cost_functor(cam_intrinsic, target_corner.position_local,
                                   observed_kp);
        double residuals[2];
        cost_functor(trans_W_Cb.data(), rot_W_Cb.coeffs().data(),
                     trans_Cb_Ci.data(),
                     rot_Cb_Ci.unit_quaternion().coeffs().data(),
                     cam_intrinsic->parameters.data(), trans_W_Ti.data(),
                     rot_W_Ti.unit_quaternion().coeffs().data(), residuals);

        double err_norm = std::hypot(residuals[0], residuals[1]);
        if (err_norm > threshold) {
          outlier_ids.push_back(kp_id);
        }
      }

      // Remove outlier keypoints from the frame.
      for (int kp_id : outlier_ids) {
        frame_ptr->keypoints.erase(kp_id);
      }
      total_removed += outlier_ids.size();
    }
  }

  return total_removed;
}

bool ReprojEvaluator::EvaluateReprojError(
    const ReprojErrorPerFrame& reproj_errors, const std::string& work_dir) {
  if (reproj_errors.empty()) {
    spdlog::error("No reprojection errors to evaluate.");
    return false;
  }

  // ==========================================================================

  // Step 1: Reorganize and check reprojection errors.
  // Containers for 2D reprojection errors (with sign) per camera and overall.
  std::map<std::string, std::vector<cv::Point2d>> reproj_error_per_cam;
  std::vector<cv::Point2d> all_reproj_error;
  const auto& cam_configs = sensor_manager_->GetAllCamConfigs();

  // Create per-camera output directories first.
  for (const auto& [cam_label, _] : cam_configs) {
    const std::string cam_dir = work_dir + "/" + cam_label;
    if (!std::filesystem::exists(cam_dir)) {
      std::filesystem::create_directories(cam_dir);
    }
  }

  constexpr double kLargeErrorThreshold = 10.0;  // Pixels

  for (const auto& [cam_label, frame_map] : reproj_errors) {
    for (const auto& [frame_ptr, errors] : frame_map) {
      if (errors.empty()) continue;

      for (const auto& err : errors) {
        reproj_error_per_cam[cam_label].push_back(err);
        all_reproj_error.push_back(err);

        double err_norm = std::hypot(err.x, err.y);
        if (err_norm > kLargeErrorThreshold) {
          spdlog::warn(
              "Large reprojection error ({:.2f} px) in camera '{}', time "
              "{:.6f}. Image: {}",
              err_norm, cam_label, frame_ptr->timestamp, frame_ptr->img_path);
        }
      }
    }
  }

  // ===========================================================================

  // Step 2: Save per-camera all-frame statistics and plots.
  struct CameraSummary {
    int num_frames = 0;
    ErrorStats stats;
  };
  std::map<std::string, CameraSummary> camera_summaries;

  for (const auto& [cam_label, _] : cam_configs) {
    const auto& errors_2d = reproj_error_per_cam[cam_label];
    const ErrorStats stats = ComputeErrorStats(errors_2d);
    int num_frames = 0;
    if (reproj_errors.count(cam_label)) {
      num_frames = static_cast<int>(reproj_errors.at(cam_label).size());
    }

    camera_summaries[cam_label] = {num_frames, stats};

    const std::string cam_dir = work_dir + "/" + cam_label;
    const std::string txt_file_path =
        cam_dir + "/" + cam_label + "_all_reproj_error.txt";
    if (!SaveErrorDataTxt(txt_file_path, cam_label, num_frames, stats,
                          errors_2d, true)) {
      spdlog::warn("Failed to save reprojection error data: {}", txt_file_path);
    }

    const std::string png_file_path =
        cam_dir + "/" + cam_label + "_all_reproj_error.png";
    if (!errors_2d.empty() && !DrawErrorPlot(cam_label, errors_2d, stats,
                                             num_frames, png_file_path)) {
      spdlog::warn("Failed to save reprojection error plot: {}", png_file_path);
    }

    spdlog::info(
        "Camera {}: mean={:.4f}px, median={:.4f}px, min={:.4f}px, max={:.4f}px",
        cam_label, stats.mean, stats.median, stats.min, stats.max);
  }

  // ==========================================================================

  // Step 3: Save overall statistics and plot (keep original naming).
  const std::string overall_dir = work_dir + "/overall";
  if (!std::filesystem::exists(overall_dir)) {
    std::filesystem::create_directories(overall_dir);
  }

  int total_frames = 0;
  for (const auto& [_, frame_map] : reproj_errors) {
    total_frames += static_cast<int>(frame_map.size());
  }

  const ErrorStats overall_stats = ComputeErrorStats(all_reproj_error);
  const std::string overall_txt = overall_dir + "/overall_reproj_error.txt";
  if (!SaveErrorDataTxt(overall_txt, "overall", total_frames, overall_stats,
                        all_reproj_error, false)) {
    spdlog::warn("Failed to save reprojection error data: {}", overall_txt);
  }

  const std::string overall_png = overall_dir + "/overall_reproj_error.png";
  if (!all_reproj_error.empty() &&
      !DrawErrorPlot("overall", all_reproj_error, overall_stats, total_frames,
                     overall_png)) {
    spdlog::warn("Failed to save reprojection error plot: {}", overall_png);
  }

  spdlog::info(
      "Overall: n={}, mean={:.4f}px, median={:.4f}px, min={:.4f}px, "
      "max={:.4f}px",
      all_reproj_error.size(), overall_stats.mean, overall_stats.median,
      overall_stats.min, overall_stats.max);

  // Step 4: Save per-camera summary in overall directory.
  const std::string summary_txt = overall_dir + "/camera_reproj_summary.txt";
  std::ofstream summary_file(summary_txt);
  if (!summary_file.is_open()) {
    spdlog::warn("Failed to save per-camera summary: {}", summary_txt);
  } else {
    summary_file << "# Reprojection summary (overall + per camera)\n";
    summary_file << "total_frames = " << total_frames << "\n";
    summary_file << "total_points = " << overall_stats.count << "\n";
    summary_file << "overall_mean_px = " << std::fixed << std::setprecision(6)
                 << overall_stats.mean << "\n";
    summary_file << "overall_median_px = " << std::fixed << std::setprecision(6)
                 << overall_stats.median << "\n";
    summary_file << "overall_min_px = " << std::fixed << std::setprecision(6)
                 << overall_stats.min << "\n";
    summary_file << "overall_max_px = " << std::fixed << std::setprecision(6)
                 << overall_stats.max << "\n";
    summary_file << "camera_count = " << camera_summaries.size() << "\n";
    for (const auto& [cam_label, summary] : camera_summaries) {
      summary_file << cam_label << ": frames=" << summary.num_frames
                   << ", points=" << summary.stats.count
                   << ", mean_px=" << std::fixed << std::setprecision(6)
                   << summary.stats.mean
                   << ", median_px=" << summary.stats.median
                   << ", min_px=" << summary.stats.min
                   << ", max_px=" << summary.stats.max << "\n";
    }
  }

  // ==========================================================================

  return true;
}

bool ReprojEvaluator::SaveReprojImage(
    const CamPoses& cam_poses_W_Cb,
    const FrameCorrespondence& frame_correspondence,
    const std::string& work_dir) {
  if (cam_poses_W_Cb.empty() || frame_correspondence.empty()) {
    spdlog::warn(
        "Empty camera poses or frame correspondence for reprojection "
        "visualization.");
    return false;
  }

  spdlog::info("Saving reprojection visualization images...");

  const auto& target_corners = sensor_manager_->GetTargetCorners()->corners;
  const auto& cam_configs = sensor_manager_->GetAllCamConfigs();

  // ===========================================================================

  // Step 1: Collect candidate frames for each camera.
  std::map<std::string, std::vector<std::pair<int, CamFrame::Ptr>>>
      cam_frame_candidates;
  for (const auto& [frame_idx, cam_frames_map] : frame_correspondence) {
    if (cam_poses_W_Cb.find(frame_idx) == cam_poses_W_Cb.end()) {
      continue;
    }
    for (const auto& [cam_label, frame_ptr] : cam_frames_map) {
      if (!frame_ptr || frame_ptr->keypoints.empty()) {
        continue;
      }
      cam_frame_candidates[cam_label].emplace_back(frame_idx, frame_ptr);
    }
  }

  // ===========================================================================

  // Step 2: Save per-frame reprojection images and statistics for each camera.
  const std::string kPerFrameDirName = "per_frame_reprojection_validation";
  bool disable_reproj_image_saving = false;
  for (const auto& [cam_label, cam_config] : cam_configs) {
    const std::string cam_dir = work_dir + "/" + cam_label;
    if (!std::filesystem::exists(cam_dir)) {
      std::filesystem::create_directories(cam_dir);
    }

    const std::string per_frame_dir = cam_dir + "/" + kPerFrameDirName;
    if (!std::filesystem::exists(per_frame_dir)) {
      std::filesystem::create_directories(per_frame_dir);
    }

    const std::string per_frame_txt =
        per_frame_dir + "/per_frame_reproj_stats.txt";
    std::ofstream stats_file(per_frame_txt);
    if (!stats_file.is_open()) {
      spdlog::warn("Failed to save per-frame reprojection stats: {}",
                   per_frame_txt);
      continue;
    }
    stats_file << "# Per-frame reprojection statistics for camera: "
               << cam_label << "\n";

    auto candidates_it = cam_frame_candidates.find(cam_label);
    if (candidates_it == cam_frame_candidates.end() ||
        candidates_it->second.empty()) {
      continue;
    }

    const auto& candidates = candidates_it->second;
    const bool save_all_frames = cam_config.save_all_reproj_image;
    const size_t kSampleCount = 10;
    const size_t target_count = save_all_frames
                                    ? candidates.size()
                                    : std::min(kSampleCount, candidates.size());
    const auto selected_indices =
        SelectUniformIndices(candidates.size(), target_count);

    // ==========================================================================

    // Step 2.1: Prepare pose and camera parameters for this frame.
    for (const size_t candidate_idx : selected_indices) {
      const auto& [frame_idx, frame_ptr] = candidates[candidate_idx];
      auto pose_it = cam_poses_W_Cb.find(frame_idx);
      if (pose_it == cam_poses_W_Cb.end()) {
        continue;
      }

      const Eigen::Vector3d& trans_W_Cb = pose_it->second.first;
      const Eigen::Quaterniond& rot_W_Cb = pose_it->second.second;
      const auto& trans_Cb_Ci = calib_parameters_->trans_Cb_Ci.at(cam_label);
      const auto& rot_Cb_Ci = calib_parameters_->rot_Cb_Ci.at(cam_label);
      const auto& cam_intrinsic =
          calib_parameters_->cam_intrinsics.at(cam_label);

      Eigen::Vector3d trans_W_Ci = rot_W_Cb * trans_Cb_Ci + trans_W_Cb;
      Eigen::Quaterniond rot_W_Ci = rot_W_Cb * rot_Cb_Ci.unit_quaternion();

      // ==========================================================================

      // Step 2.2: Try loading image; if failed once, disable drawing/saving but
      // keep statistics computation.
      cv::Mat viz_img;
      int obs_radius = 4;
      int reproj_radius = 3;
      int line_thickness = 1;
      if (!disable_reproj_image_saving) {
        cv::Mat img = cv::imread(frame_ptr->img_path);
        if (img.empty()) {
          spdlog::warn(
              "Failed to read image for reprojection visualization: {}. "
              "Stop reprojection image drawing and saving, but continue "
              "statistics.",
              frame_ptr->img_path);
          disable_reproj_image_saving = true;
        } else {
          viz_img = img.clone();
          const int image_size = std::min(img.rows, img.cols);
          obs_radius = std::max(4, static_cast<int>(image_size * 0.005));
          reproj_radius = std::max(3, static_cast<int>(image_size * 0.003));
          line_thickness = std::max(1, static_cast<int>(image_size * 0.0015));
        }
      }

      // ==========================================================================

      // Step 2.3: Project points, draw reprojection overlays, and collect
      // frame-wise reprojection errors.
      std::vector<double> frame_errors;
      frame_errors.reserve(frame_ptr->keypoints.size());

      for (const auto& [kp_id, observed_kp] : frame_ptr->keypoints) {
        if (target_corners.count(kp_id) == 0) continue;

        const auto& target_corner = target_corners.at(kp_id);
        int target_idx = target_corner.target_idx;
        if (calib_parameters_->trans_W_Ti.count(target_idx) == 0) continue;

        const auto& trans_W_Ti = calib_parameters_->trans_W_Ti.at(target_idx);
        const auto& rot_W_Ti = calib_parameters_->rot_W_Ti.at(target_idx);
        Eigen::Vector3d p_W =
            rot_W_Ti * target_corner.position_local + trans_W_Ti;

        Eigen::Vector2d projected_point_2d;
        bool projected = false;
        if (auto ptr =
                std::dynamic_pointer_cast<CamRadtanIntrinsic>(cam_intrinsic)) {
          CamRadtanIntrinsic::Space2Image(trans_W_Ci, rot_W_Ci, p_W,
                                          cam_intrinsic->parameters.data(),
                                          projected_point_2d);
          projected = true;
        } else if (auto ptr = std::dynamic_pointer_cast<CamEqdistIntrinsic>(
                       cam_intrinsic)) {
          CamEqdistIntrinsic::Space2Image(trans_W_Ci, rot_W_Ci, p_W,
                                          cam_intrinsic->parameters.data(),
                                          projected_point_2d);
          projected = true;
        }

        if (projected) {
          cv::Point2d reproj_pt(projected_point_2d.x(), projected_point_2d.y());
          cv::Point2d obs_pt(observed_kp.x(), observed_kp.y());

          if (!viz_img.empty()) {
            // Draw reprojected point as a filled dot.
            cv::circle(viz_img, reproj_pt, reproj_radius, cv::Scalar(0, 0, 255),
                       -1);

            // Draw connection line.
            cv::line(viz_img, obs_pt, reproj_pt, cv::Scalar(0, 255, 255),
                     line_thickness);
          }

          frame_errors.push_back(
              std::hypot(reproj_pt.x - obs_pt.x, reproj_pt.y - obs_pt.y));
        }
      }

      // ==========================================================================

      // Step 2.4: Draw observed points as thin crosses on top of all overlays.
      if (!viz_img.empty()) {
        const int obs_cross_half = std::max(3, obs_radius);
        const int obs_cross_thickness = 1;
        for (const auto& [kp_id, observed_kp] : frame_ptr->keypoints) {
          cv::Point2d obs_pt(observed_kp.x(), observed_kp.y());
          cv::line(viz_img, cv::Point2d(obs_pt.x - obs_cross_half, obs_pt.y),
                   cv::Point2d(obs_pt.x + obs_cross_half, obs_pt.y),
                   cv::Scalar(255, 0, 0), obs_cross_thickness);
          cv::line(viz_img, cv::Point2d(obs_pt.x, obs_pt.y - obs_cross_half),
                   cv::Point2d(obs_pt.x, obs_pt.y + obs_cross_half),
                   cv::Scalar(255, 0, 0), obs_cross_thickness);
        }
      }

      // ==========================================================================

      // Step 2.5: Compute per-frame reprojection statistics.
      double mean_err = 0.0;
      double median_err = 0.0;
      double min_err = 0.0;
      double max_err = 0.0;
      if (!frame_errors.empty()) {
        min_err = std::numeric_limits<double>::max();
        max_err = 0.0;
        double sum_err = 0.0;
        for (const double err : frame_errors) {
          sum_err += err;
          min_err = std::min(min_err, err);
          max_err = std::max(max_err, err);
        }
        std::sort(frame_errors.begin(), frame_errors.end());
        mean_err = sum_err / static_cast<double>(frame_errors.size());
        if (frame_errors.size() % 2 == 1) {
          median_err = frame_errors[frame_errors.size() / 2];
        } else {
          const size_t mid = frame_errors.size() / 2;
          median_err = 0.5 * (frame_errors[mid - 1] + frame_errors[mid]);
        }
      }

      // ==========================================================================

      // Step 2.6: Save visualization image and one-line frame statistics.
      std::ostringstream ts_ss;
      ts_ss << std::fixed << std::setprecision(6) << frame_ptr->timestamp;
      std::string ts_str = ts_ss.str();
      std::string filename =
          std::filesystem::path(frame_ptr->img_path).filename().string();
      if (!viz_img.empty()) {
        std::string save_path = per_frame_dir + "/" + filename;
        if (!cv::imwrite(save_path, viz_img)) {
          spdlog::warn("Failed to save reprojection visualization image: {}",
                       save_path);
        }
      }

      stats_file << "image=" << filename << ", timestamp=" << ts_str
                 << ", points=" << frame_errors.size()
                 << ", mean_px=" << std::fixed << std::setprecision(6)
                 << mean_err << ", median_px=" << median_err
                 << ", min_px=" << min_err << ", max_px=" << max_err << "\n";
    }
  }

  // ===========================================================================

  return true;
}

}  // namespace xr_ucalib

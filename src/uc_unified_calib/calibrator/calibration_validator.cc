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

#include "xr_ucalib/uc_unified_calib/calibrator/calibration_validator.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <random>
#include <set>
#include <sstream>
#include <tuple>
#include <vector>

#include "xr_ucalib/uc_unified_calib/cost_function/cam_reproj_ct_cost.hpp"
#include "xr_ucalib/uc_unified_calib/cost_function/imu_acc_cost.hpp"
#include "xr_ucalib/uc_unified_calib/cost_function/imu_gyr_cost.hpp"

namespace xr_ucalib {

namespace {

/// @brief Per-sequence completeness statistics within a given evaluation time
/// window.
struct SequenceCompletenessMetrics {
  double completeness = 0.;
  double total_duration = 0.;
  double missing_duration = 0.;
  size_t valid_sample_count = 0;
  std::vector<std::pair<double, double>> missing_intervals;
};

/// @brief Reprojection error statistics.
struct ReprojStats {
  size_t count = 0;
  double mean = 0.0;
  double median = 0.0;
  double min = 0.0;
  double max = 0.0;
};

/// @brief Merge overlapping/adjacent time intervals and return a compact
/// interval set.
std::vector<std::pair<double, double>> MergeIntervals(
    std::vector<std::pair<double, double>> intervals) {
  if (intervals.empty()) {
    return {};
  }

  std::sort(intervals.begin(), intervals.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  std::vector<std::pair<double, double>> merged;
  merged.reserve(intervals.size());
  merged.push_back(intervals.front());
  for (size_t i = 1; i < intervals.size(); ++i) {
    auto& back = merged.back();
    if (intervals[i].first <= back.second) {
      back.second = std::max(back.second, intervals[i].second);
    } else {
      merged.push_back(intervals[i]);
    }
  }

  return merged;
}

/// @brief Intersect two interval sets.
std::vector<std::pair<double, double>> IntersectIntervals(
    const std::vector<std::pair<double, double>>& lhs,
    const std::vector<std::pair<double, double>>& rhs) {
  std::vector<std::pair<double, double>> result;
  if (lhs.empty() || rhs.empty()) {
    return result;
  }

  auto a = MergeIntervals(lhs);
  auto b = MergeIntervals(rhs);
  size_t i = 0;
  size_t j = 0;
  while (i < a.size() && j < b.size()) {
    const double s = std::max(a[i].first, b[j].first);
    const double e = std::min(a[i].second, b[j].second);
    if (s < e) {
      result.emplace_back(s, e);
    }

    if (a[i].second < b[j].second) {
      ++i;
    } else {
      ++j;
    }
  }

  return result;
}

bool IsCameraSensorKey(const std::string& sensor_key) {
  return sensor_key.rfind("cam:", 0) == 0;
}

/// @brief Compute sequence completeness by detecting long gaps (larger than
/// threshold) in [start_time, end_time].
SequenceCompletenessMetrics ComputeSequenceCompleteness(
    const std::vector<double>& timestamps, const double start_time,
    const double end_time, const double missing_threshold_sec = 1.0) {
  SequenceCompletenessMetrics metrics;
  metrics.total_duration = std::max(0.0, end_time - start_time);

  if (metrics.total_duration <= 0.) {
    return metrics;
  }

  // Keep only finite timestamps inside the evaluation window.
  std::vector<double> valid_ts;
  valid_ts.reserve(timestamps.size());
  for (const double t : timestamps) {
    if (std::isfinite(t) && t >= start_time && t <= end_time) {
      valid_ts.push_back(t);
    }
  }

  if (valid_ts.empty()) {
    metrics.missing_intervals.emplace_back(start_time, end_time);
    metrics.missing_duration = metrics.total_duration;
    metrics.completeness = 0.;
    return metrics;
  }

  // Sort and de-duplicate before gap analysis.
  std::sort(valid_ts.begin(), valid_ts.end());
  valid_ts.erase(std::unique(valid_ts.begin(), valid_ts.end()), valid_ts.end());
  metrics.valid_sample_count = valid_ts.size();

  // Head gap.
  if ((valid_ts.front() - start_time) > missing_threshold_sec) {
    metrics.missing_intervals.emplace_back(start_time, valid_ts.front());
  }

  // Internal gaps.
  for (size_t i = 1; i < valid_ts.size(); ++i) {
    if ((valid_ts[i] - valid_ts[i - 1]) > missing_threshold_sec) {
      metrics.missing_intervals.emplace_back(valid_ts[i - 1], valid_ts[i]);
    }
  }

  // Tail gap.
  if ((end_time - valid_ts.back()) > missing_threshold_sec) {
    metrics.missing_intervals.emplace_back(valid_ts.back(), end_time);
  }

  metrics.missing_intervals = MergeIntervals(metrics.missing_intervals);
  for (const auto& [s, e] : metrics.missing_intervals) {
    metrics.missing_duration += std::max(0.0, e - s);
  }

  metrics.missing_duration =
      std::min(metrics.missing_duration, metrics.total_duration);
  metrics.completeness = std::clamp(
      1. - metrics.missing_duration / metrics.total_duration, 0., 1.);

  return metrics;
}

/// @brief RMSE helper; returns -1 when input is empty.
double ComputeRmse(const std::vector<double>& values) {
  if (values.empty()) {
    return -1.;
  }
  double sq_sum = std::accumulate(values.begin(), values.end(), 0.0,
                                  [](double s, double v) { return s + v * v; });
  return std::sqrt(sq_sum / static_cast<double>(values.size()));
}

/// @brief Compute reprojection statistics from 2D residual vectors.
ReprojStats ComputeReprojStats(
    const std::vector<std::pair<double, double>>& errors_2d) {
  ReprojStats stats;
  if (errors_2d.empty()) {
    return stats;
  }

  std::vector<double> mags;
  mags.reserve(errors_2d.size());
  double sum = 0.0;
  for (const auto& [ex, ey] : errors_2d) {
    const double dist = std::hypot(ex, ey);
    mags.push_back(dist);
    sum += dist;
  }

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
    const std::vector<std::pair<double, double>>& errors_2d) {
  if (errors_2d.empty()) {
    return {cv::Point2d(0.0, 0.0), 0.0, 0.0, 0.0};
  }

  cv::Point2d mean(0.0, 0.0);
  for (const auto& [ex, ey] : errors_2d) {
    mean.x += ex;
    mean.y += ey;
  }
  mean.x /= errors_2d.size();
  mean.y /= errors_2d.size();

  double cov_xx = 0.0, cov_yy = 0.0, cov_xy = 0.0;
  for (const auto& [ex, ey] : errors_2d) {
    const double dx = ex - mean.x;
    const double dy = ey - mean.y;
    cov_xx += dx * dx;
    cov_yy += dy * dy;
    cov_xy += dx * dy;
  }
  cov_xx /= errors_2d.size();
  cov_yy /= errors_2d.size();
  cov_xy /= errors_2d.size();

  return {mean, cov_xx, cov_yy, cov_xy};
}

/// @brief Save reprojection error data and summary to a text file.
bool SaveReprojErrorTxt(const std::string& txt_file_path,
                        const std::string& title_label, int num_frames,
                        const ReprojStats& stats,
                        const std::vector<std::pair<double, double>>& errors_2d,
                        bool plain_stat_lines) {
  std::ofstream txt_file(txt_file_path);
  if (!txt_file.is_open()) {
    return false;
  }

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

  for (const auto& [ex, ey] : errors_2d) {
    txt_file << std::fixed << std::setprecision(6) << ex << " " << ey << "\n";
  }

  return true;
}

/// @brief Draw reprojection error distribution and save it as a png image.
/// @param cam_label Camera label used in the figure title.
/// @param errors_2d 2D reprojection residual vectors (error_x, error_y).
/// @param stats Scalar statistics (mean/median/min/max) of residual magnitudes.
/// @param num_frames Number of frames contributing to current residual set.
/// @param file_path Output image path.
/// @return True if image is generated and saved successfully; false otherwise.
bool DrawReprojPlot(const std::string& cam_label,
                    const std::vector<std::pair<double, double>>& errors_2d,
                    const ReprojStats& stats, int num_frames,
                    const std::string& file_path) {
  if (errors_2d.empty()) {
    return false;
  }

  // ===========================================================================

  // Step 1: Prepare canvas, statistics, and plotting scale.
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
    const double sqrt_disc = std::sqrt(discriminant);
    const double lambda1 = std::max(trace / 2.0 + sqrt_disc, 1e-6);
    sigma_3_max = 3.0 * std::sqrt(lambda1);
  }

  double max_range = std::max({stats.max * 1.2, sigma_3_max * 1.2, 1.0});
  max_range = std::ceil(max_range);
  const double scale = (plot_size / 2.0) / max_range;

  // ===========================================================================

  // Step 2: Draw grid and axes.
  cv::Mat img(img_size, img_size, CV_8UC3, cv::Scalar(255, 255, 255));
  const cv::Scalar grid_color(220, 220, 220);
  for (int i = -4; i <= 4; ++i) {
    const int offset = static_cast<int>(i * (plot_size / 8.0));
    cv::line(img, cv::Point(center.x + offset, margin),
             cv::Point(center.x + offset, img_size - margin), grid_color, 1);
    cv::line(img, cv::Point(margin, center.y + offset),
             cv::Point(img_size - margin, center.y + offset), grid_color, 1);
  }

  const cv::Scalar axis_color(100, 100, 100);
  cv::line(img, cv::Point(margin, center.y),
           cv::Point(img_size - margin, center.y), axis_color, 2);
  cv::line(img, cv::Point(center.x, margin),
           cv::Point(center.x, img_size - margin), axis_color, 2);
  cv::putText(img, "Error X (px)",
              cv::Point(img_size - margin - 90, center.y + 45),
              cv::FONT_HERSHEY_SIMPLEX, 0.55, axis_color, 1.5);
  cv::putText(img, "Error Y (px)", cv::Point(center.x - 110, margin + 6),
              cv::FONT_HERSHEY_SIMPLEX, 0.55, axis_color, 1.5);

  // ===========================================================================

  // Step 3: Draw residual points, mean point, and 3-sigma ellipse.
  const cv::Scalar point_color(255, 0, 0);
  const int cross_size = 3;
  for (const auto& [ex, ey] : errors_2d) {
    const int px = center.x + static_cast<int>(ex * scale);
    const int py = center.y - static_cast<int>(ey * scale);
    if (px < margin || px > img_size - margin || py < margin ||
        py > img_size - margin) {
      continue;
    }
    cv::line(img, cv::Point(px - cross_size, py - cross_size),
             cv::Point(px + cross_size, py + cross_size), point_color, 1);
    cv::line(img, cv::Point(px - cross_size, py + cross_size),
             cv::Point(px + cross_size, py - cross_size), point_color, 1);
  }

  const cv::Point2i mean_pt(center.x + static_cast<int>(mean_2d.x * scale),
                            center.y - static_cast<int>(mean_2d.y * scale));
  cv::circle(img, mean_pt, 5, cv::Scalar(0, 255, 0), -1);

  if (discriminant >= 0) {
    const double sqrt_disc = std::sqrt(discriminant);
    const double lambda1 = std::max(trace / 2.0 + sqrt_disc, 1e-6);
    const double lambda2 = std::max(trace / 2.0 - sqrt_disc, 1e-6);
    double angle = 0.0;
    if (std::abs(cov_xy) > 1e-10) {
      angle = 0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);
    }
    const double axis_a = 3.0 * std::sqrt(lambda1) * scale;
    const double axis_b = 3.0 * std::sqrt(lambda2) * scale;
    cv::ellipse(img, mean_pt,
                cv::Size(static_cast<int>(axis_a), static_cast<int>(axis_b)),
                -angle * 180.0 / M_PI, 0, 360, cv::Scalar(0, 0, 255), 2);
  }

  // ===========================================================================

  // Step 4: Draw textual summary and save figure.
  int text_y = 25;
  cv::putText(img,
              cv::format("Camera: %s, from %d frames, %zu points",
                         cam_label.c_str(), num_frames, errors_2d.size()),
              cv::Point(10, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.6,
              cv::Scalar(0, 0, 0), 2);
  text_y += 25;
  cv::putText(img,
              cv::format("mean=%.2fpx, median=%.2fpx, min=%.2fpx, max=%.2fpx",
                         stats.mean, stats.median, stats.min, stats.max),
              cv::Point(10, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(0, 0, 0), 1.5);

  // ===========================================================================

  return cv::imwrite(file_path, img);
}

/// @brief Randomly select a subset of indices from [0, total_count) without
/// replacement.
std::vector<size_t> SelectRandomIndices(size_t total_count,
                                        size_t sample_count) {
  std::vector<size_t> indices;
  if (total_count == 0) {
    return indices;
  }

  indices.resize(total_count);
  for (size_t i = 0; i < total_count; ++i) {
    indices[i] = i;
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::shuffle(indices.begin(), indices.end(), gen);

  if (sample_count < indices.size()) {
    indices.resize(sample_count);
  }
  std::sort(indices.begin(), indices.end());
  return indices;
}

}  // namespace

bool CalibrationValidator::CalImuResidual(const std::string& label,
                                          const ImuFrame::Ptr& imu_frame,
                                          double& acc_res_norm,
                                          double& gyr_res_norm) {
  acc_res_norm = 0.;
  gyr_res_norm = 0.;

  if (!imu_frame) {
    spdlog::error("Null IMU frame pointer for residual calculation [{}].",
                  label);
    return false;
  }

  // ===========================================================================

  // Step 1: Calculate spline meta data for this measurement timestamp.
  SplineMeta<kSplineOrder> trans_meta, rot_meta;
  double meta_min_time = -1.;
  double meta_max_time = -1.;
  const double kMeasTimestamp = imu_frame->timestamp;

  if (!CalMetaMinMaxTime(kMeasTimestamp, meta_min_time, meta_max_time)) {
    return false;
  }

  const auto& spline_bundle = context_.spline_bundle;
  const auto& trans_spline_name = context_.trans_spline_info.name;
  const auto& rot_spline_name = context_.rot_spline_info.name;
  if (!spline_bundle->TimeInRangeForR3dSpline(meta_min_time,
                                              trans_spline_name) ||
      !spline_bundle->TimeInRangeForR3dSpline(meta_max_time,
                                              trans_spline_name) ||
      !spline_bundle->TimeInRangeForSo3dSpline(meta_min_time,
                                               rot_spline_name) ||
      !spline_bundle->TimeInRangeForSo3dSpline(meta_max_time,
                                               rot_spline_name)) {
    spdlog::critical(
        "The min/max times of the spline meta exception for [{}] at {:.9f} ",
        label, imu_frame->timestamp);
    return false;
  }

  spline_bundle->CalculateR3dSplineMeta(
      trans_spline_name, {{meta_min_time, meta_max_time}}, trans_meta);
  spline_bundle->CalculateSo3dSplineMeta(
      rot_spline_name, {{meta_min_time, meta_max_time}}, rot_meta);

  // ===========================================================================

  // Step 2: Create unweighted cost functions to keep physical residual units.
  constexpr double kPhysicalWeight = 1.0;
  auto* imu_acc_cost = ImuAccCost<kSplineOrder>::Create(
      trans_meta, rot_meta, imu_frame, kPhysicalWeight);
  auto* imu_gyr_cost =
      ImuGyrCost<kSplineOrder>::Create(rot_meta, imu_frame, kPhysicalWeight);

  // Parameter blocks for spline knots.
  for (size_t i = 0; i < trans_meta.NumParameters(); ++i) {
    imu_acc_cost->AddParameterBlock(3);
  }
  for (size_t i = 0; i < rot_meta.NumParameters(); ++i) {
    imu_acc_cost->AddParameterBlock(4);
    imu_gyr_cost->AddParameterBlock(4);
  }

  // Add static parameters for accelerometer model.
  imu_acc_cost->AddParameterBlock(3);  // trans_B_Ii
  imu_acc_cost->AddParameterBlock(4);  // rot_B_Ii
  imu_acc_cost->AddParameterBlock(3);  // gravity_in_W
  imu_acc_cost->AddParameterBlock(1);  // toff_B_Ii
  imu_acc_cost->AddParameterBlock(3);  // acc_bias
  imu_acc_cost->AddParameterBlock(3);  // acc_scale
  imu_acc_cost->AddParameterBlock(3);  // acc_non_ortho
  imu_acc_cost->SetNumResiduals(3);

  // Add static parameters for gyroscope model.
  imu_gyr_cost->AddParameterBlock(4);  // rot_B_Ii
  imu_gyr_cost->AddParameterBlock(1);  // toff_B_Ii
  imu_gyr_cost->AddParameterBlock(3);  // gyr_bias
  imu_gyr_cost->AddParameterBlock(3);  // gyr_scale
  imu_gyr_cost->AddParameterBlock(3);  // gyr_non_ortho
  imu_gyr_cost->AddParameterBlock(4);  // rot_gyr_acc
  imu_gyr_cost->SetNumResiduals(3);

  // ===========================================================================

  // Step 3: Build parameter block vectors for direct Evaluate.
  std::vector<double*> acc_param_block_vector;
  std::vector<double*> gyr_param_block_vector;
  acc_param_block_vector.reserve(trans_meta.NumParameters() +
                                 rot_meta.NumParameters() + 7);
  gyr_param_block_vector.reserve(rot_meta.NumParameters() + 6);

  // Add R3 spline knots for accelerometer factor.
  const auto& trans_spline = spline_bundle->GetR3dSpline(trans_spline_name);
  for (const auto& segment : trans_meta.segments) {
    size_t index;
    double fraction;
    trans_spline.ComputeTimeIndex(
        segment.start_time + segment.knot_interval * 0.5, index, fraction);
    for (std::size_t i = index; i < index + segment.NumParameters(); ++i) {
      auto* data = const_cast<double*>(
          trans_spline.get_knot(static_cast<int>(i)).data());
      acc_param_block_vector.push_back(data);
    }
  }

  // Add SO3 spline knots for both acc and gyr factors.
  const auto& rot_spline = spline_bundle->GetSo3dSpline(rot_spline_name);
  for (const auto& segment : rot_meta.segments) {
    size_t index;
    double fraction;
    rot_spline.ComputeTimeIndex(
        segment.start_time + segment.knot_interval * 0.5, index, fraction);
    for (std::size_t i = index; i < index + segment.NumParameters(); ++i) {
      auto* data =
          const_cast<double*>(rot_spline.get_knot(static_cast<int>(i)).data());
      acc_param_block_vector.push_back(data);
      gyr_param_block_vector.push_back(data);
    }
  }

  auto trans_B_Ii_ptr = context_.calib_parameters->trans_B_Ii.at(label).data();
  auto rot_B_Ii_ptr = context_.calib_parameters->rot_B_Ii.at(label).data();
  auto gravity_in_W_ptr = context_.calib_parameters->gravity_in_W.data();
  auto toff_B_Ii_ptr = &context_.calib_parameters->time_offset_B_Ii.at(label);
  auto acc_bias_ptr =
      context_.calib_parameters->imu_intrinsics.at(label)->acc_bias.data();
  auto acc_scale_ptr =
      context_.calib_parameters->imu_intrinsics.at(label)->acc_scale.data();
  auto acc_non_ortho_ptr = context_.calib_parameters->imu_intrinsics.at(label)
                               ->acc_non_orthogonal.data();
  auto gyr_bias_ptr =
      context_.calib_parameters->imu_intrinsics.at(label)->gyr_bias.data();
  auto gyr_scale_ptr =
      context_.calib_parameters->imu_intrinsics.at(label)->gyr_scale.data();
  auto gyr_non_ortho_ptr = context_.calib_parameters->imu_intrinsics.at(label)
                               ->gyr_non_orthogonal.data();
  auto rot_gyr_acc_ptr =
      context_.calib_parameters->imu_intrinsics.at(label)->rot_gyr_acc.data();

  acc_param_block_vector.push_back(trans_B_Ii_ptr);
  acc_param_block_vector.push_back(rot_B_Ii_ptr);
  acc_param_block_vector.push_back(gravity_in_W_ptr);
  acc_param_block_vector.push_back(toff_B_Ii_ptr);
  acc_param_block_vector.push_back(acc_bias_ptr);
  acc_param_block_vector.push_back(acc_scale_ptr);
  acc_param_block_vector.push_back(acc_non_ortho_ptr);

  gyr_param_block_vector.push_back(rot_B_Ii_ptr);
  gyr_param_block_vector.push_back(toff_B_Ii_ptr);
  gyr_param_block_vector.push_back(gyr_bias_ptr);
  gyr_param_block_vector.push_back(gyr_scale_ptr);
  gyr_param_block_vector.push_back(gyr_non_ortho_ptr);
  gyr_param_block_vector.push_back(rot_gyr_acc_ptr);

  // ===========================================================================

  // Step 4: Evaluate residual vectors and convert to scalar norms.
  std::array<double, 3> acc_residual = {0., 0., 0.};
  std::array<double, 3> gyr_residual = {0., 0., 0.};

  const bool acc_ok = imu_acc_cost->Evaluate(acc_param_block_vector.data(),
                                             acc_residual.data(), nullptr);
  const bool gyr_ok = imu_gyr_cost->Evaluate(gyr_param_block_vector.data(),
                                             gyr_residual.data(), nullptr);

  delete imu_acc_cost;
  delete imu_gyr_cost;

  if (!acc_ok || !gyr_ok) {
    spdlog::error("Failed to evaluate IMU residual for [{}] at {:.9f}.", label,
                  imu_frame->timestamp);
    return false;
  }

  acc_res_norm = std::sqrt(acc_residual[0] * acc_residual[0] +
                           acc_residual[1] * acc_residual[1] +
                           acc_residual[2] * acc_residual[2]);
  gyr_res_norm = std::sqrt(gyr_residual[0] * gyr_residual[0] +
                           gyr_residual[1] * gyr_residual[1] +
                           gyr_residual[2] * gyr_residual[2]);

  // ===========================================================================

  return true;
}

bool CalibrationValidator::CalCamReprojResidual(
    const std::string& label, const CamFrame::Ptr& cam_frame,
    std::vector<std::pair<double, double>>& residual_pairs,
    std::vector<std::pair<double, double>>& observed_points,
    std::vector<std::pair<double, double>>& reprojected_points) {
  residual_pairs.clear();
  observed_points.clear();
  reprojected_points.clear();

  if (!cam_frame) {
    spdlog::error("Null camera frame pointer for reprojection validation [{}].",
                  label);
    return false;
  }

  // ===========================================================================

  // Step 1: Build spline meta data for current frame timestamp.
  SplineMeta<kSplineOrder> trans_meta, rot_meta;
  double meta_min_time = -1.;
  double meta_max_time = -1.;
  const double kMeasTimestamp = cam_frame->timestamp;

  if (!CalMetaMinMaxTime(kMeasTimestamp, meta_min_time, meta_max_time)) {
    return false;
  }

  const auto& spline_bundle = context_.spline_bundle;
  const auto& trans_spline_name = context_.trans_spline_info.name;
  const auto& rot_spline_name = context_.rot_spline_info.name;
  if (!spline_bundle->TimeInRangeForR3dSpline(meta_min_time,
                                              trans_spline_name) ||
      !spline_bundle->TimeInRangeForR3dSpline(meta_max_time,
                                              trans_spline_name) ||
      !spline_bundle->TimeInRangeForSo3dSpline(meta_min_time,
                                               rot_spline_name) ||
      !spline_bundle->TimeInRangeForSo3dSpline(meta_max_time,
                                               rot_spline_name)) {
    spdlog::critical(
        "The min/max times of the spline meta exception for [{}] at {:.9f}",
        label, cam_frame->timestamp);
    return false;
  }

  spline_bundle->CalculateR3dSplineMeta(
      trans_spline_name, {{meta_min_time, meta_max_time}}, trans_meta);
  spline_bundle->CalculateSo3dSplineMeta(
      rot_spline_name, {{meta_min_time, meta_max_time}}, rot_meta);

  // ===========================================================================

  // Step 2: Prepare parameter blocks shared by all keypoints in this frame.
  std::vector<double*> spline_param_blocks;
  spline_param_blocks.reserve(trans_meta.NumParameters() +
                              rot_meta.NumParameters());

  const auto& trans_spline = spline_bundle->GetR3dSpline(trans_spline_name);
  for (const auto& segment : trans_meta.segments) {
    size_t index;
    double fraction;
    trans_spline.ComputeTimeIndex(
        segment.start_time + segment.knot_interval * 0.5, index, fraction);
    for (std::size_t i = index; i < index + segment.NumParameters(); ++i) {
      auto* data = const_cast<double*>(
          trans_spline.get_knot(static_cast<int>(i)).data());
      spline_param_blocks.push_back(data);
    }
  }

  const auto& rot_spline = spline_bundle->GetSo3dSpline(rot_spline_name);
  for (const auto& segment : rot_meta.segments) {
    size_t index;
    double fraction;
    rot_spline.ComputeTimeIndex(
        segment.start_time + segment.knot_interval * 0.5, index, fraction);
    for (std::size_t i = index; i < index + segment.NumParameters(); ++i) {
      auto* data =
          const_cast<double*>(rot_spline.get_knot(static_cast<int>(i)).data());
      spline_param_blocks.push_back(data);
    }
  }

  auto trans_B_Cb_ptr = context_.calib_parameters->trans_B_Cb.data();
  auto rot_B_Cb_ptr = context_.calib_parameters->rot_B_Cb.data();
  auto trans_Cb_Ci_ptr =
      context_.calib_parameters->trans_Cb_Ci.at(label).data();
  auto rot_Cb_Ci_ptr = context_.calib_parameters->rot_Cb_Ci.at(label).data();
  auto toff_B_Cb_ptr = &context_.calib_parameters->time_offset_B_Cb;
  auto toff_Cb_Ci_ptr = &context_.calib_parameters->time_offset_Cb_Ci.at(label);
  auto cam_intrinsic_ptr =
      context_.calib_parameters->cam_intrinsics.at(label)->parameters.data();
  const auto& camera_intrinsic =
      context_.calib_parameters->cam_intrinsics.at(label);
  const auto& target_corners = context_.sensor_manager->GetTargetCorners();

  // ===========================================================================

  // Step 3: Evaluate reprojection residual of each valid observed keypoint.
  for (const auto& [landmark_id, keypoint] : cam_frame->keypoints) {
    if (!target_corners->Contains(landmark_id)) {
      continue;
    }

    const auto& corner_3d = target_corners->At(landmark_id).position_local;
    const int target_idx = target_corners->At(landmark_id).target_idx;
    if (context_.calib_parameters->trans_W_Ti.count(target_idx) == 0 ||
        context_.calib_parameters->rot_W_Ti.count(target_idx) == 0) {
      continue;
    }

    auto* cam_reproj_cost = CamReprojCtCost<kSplineOrder>::Create(
        trans_meta, rot_meta, camera_intrinsic, corner_3d, keypoint,
        cam_frame->timestamp, 1.0);

    for (size_t i = 0; i < trans_meta.NumParameters(); ++i) {
      cam_reproj_cost->AddParameterBlock(3);
    }
    for (size_t i = 0; i < rot_meta.NumParameters(); ++i) {
      cam_reproj_cost->AddParameterBlock(4);
    }
    cam_reproj_cost->AddParameterBlock(3);  // trans_B_Cb
    cam_reproj_cost->AddParameterBlock(4);  // rot_B_Cb
    cam_reproj_cost->AddParameterBlock(3);  // trans_Cb_Ci
    cam_reproj_cost->AddParameterBlock(4);  // rot_Cb_Ci
    cam_reproj_cost->AddParameterBlock(3);  // trans_W_Ti
    cam_reproj_cost->AddParameterBlock(4);  // rot_W_Ti
    cam_reproj_cost->AddParameterBlock(1);  // toff_B_Cb
    cam_reproj_cost->AddParameterBlock(1);  // toff_Cb_Ci
    cam_reproj_cost->AddParameterBlock(camera_intrinsic->parameter_size);
    cam_reproj_cost->SetNumResiduals(2);

    auto trans_W_Ti_ptr =
        context_.calib_parameters->trans_W_Ti.at(target_idx).data();
    auto rot_W_Ti_ptr =
        context_.calib_parameters->rot_W_Ti.at(target_idx).data();

    std::vector<double*> param_block_vector = spline_param_blocks;
    param_block_vector.push_back(trans_B_Cb_ptr);
    param_block_vector.push_back(rot_B_Cb_ptr);
    param_block_vector.push_back(trans_Cb_Ci_ptr);
    param_block_vector.push_back(rot_Cb_Ci_ptr);
    param_block_vector.push_back(trans_W_Ti_ptr);
    param_block_vector.push_back(rot_W_Ti_ptr);
    param_block_vector.push_back(toff_B_Cb_ptr);
    param_block_vector.push_back(toff_Cb_Ci_ptr);
    param_block_vector.push_back(cam_intrinsic_ptr);

    std::array<double, 2> residual = {0., 0.};
    const bool ok = cam_reproj_cost->Evaluate(param_block_vector.data(),
                                              residual.data(), nullptr);
    delete cam_reproj_cost;

    if (!ok) {
      continue;
    }

    const double obs_x = keypoint.x();
    const double obs_y = keypoint.y();
    residual_pairs.emplace_back(residual[0], residual[1]);
    observed_points.emplace_back(obs_x, obs_y);
    reprojected_points.emplace_back(obs_x + residual[0], obs_y + residual[1]);
  }

  // ===========================================================================

  return !residual_pairs.empty();
}

bool CalibrationValidator::ValidateAndSaveResults(
    const std::string& output_path,
    const std::map<std::string, std::vector<double>>& sequence_timestamps,
    double eval_start_time, double eval_end_time, bool solution_usable) {
  if (!(eval_end_time > eval_start_time)) {
    spdlog::error("Invalid validation time range [{}, {}].", eval_start_time,
                  eval_end_time);
    return false;
  }

  std::ofstream out_file(output_path);
  if (!out_file.is_open()) {
    spdlog::error("Failed to open validation report file: {}", output_path);
    return false;
  }
  out_file << std::fixed << std::setprecision(6);

  // ===========================================================================

  // Step 1: Compute data completeness metrics per sensor.
  std::map<std::string, SequenceCompletenessMetrics> completeness_map;
  std::vector<std::pair<double, double>> camera_global_missing_intervals;
  bool has_camera_sensor = false;

  for (const auto& [sensor_key, timestamps] : sequence_timestamps) {
    auto metrics = ComputeSequenceCompleteness(timestamps, eval_start_time,
                                               eval_end_time, 1.0);
    completeness_map[sensor_key] = metrics;

    // Camera-global missing interval: missing only when all cameras miss.
    if (IsCameraSensorKey(sensor_key)) {
      if (!has_camera_sensor) {
        camera_global_missing_intervals = metrics.missing_intervals;
        has_camera_sensor = true;
      } else {
        camera_global_missing_intervals = IntersectIntervals(
            camera_global_missing_intervals, metrics.missing_intervals);
      }
    }
  }

  if (!has_camera_sensor) {
    camera_global_missing_intervals.clear();
    camera_global_missing_intervals.emplace_back(eval_start_time,
                                                 eval_end_time);
  } else {
    camera_global_missing_intervals =
        MergeIntervals(camera_global_missing_intervals);
  }

  const double total_duration = eval_end_time - eval_start_time;
  double camera_global_missing_duration = 0.;
  for (const auto& [s, e] : camera_global_missing_intervals) {
    camera_global_missing_duration += std::max(0.0, e - s);
  }
  camera_global_missing_duration =
      std::min(camera_global_missing_duration, total_duration);
  double camera_global_completeness =
      std::clamp(1. - camera_global_missing_duration / total_duration, 0., 1.);

  // ===========================================================================

  // Step 2: Compute IMU residual statistics with physical units.
  struct ImuResidualStats {
    size_t sample_count = 0;
    double acc_rmse = -1.;
    double gyr_rmse = -1.;
  };

  std::map<std::string, ImuResidualStats> imu_residual_stats;
  std::vector<double> all_acc_residuals;
  std::vector<double> all_gyr_residuals;

  const auto& imu_sequences = context_.sensor_manager->GetAllImuSequences();
  const auto& imu_configs = context_.sensor_manager->GetAllImuConfigs();
  for (const auto& [imu_label, imu_seq] : imu_sequences) {
    std::vector<double> acc_residuals;
    std::vector<double> gyr_residuals;

    const int down_sample_rate =
        imu_configs.at(imu_label).down_sample_rate_ucalib;
    int frame_idx = 0;
    for (const auto& imu_frame : imu_seq->frames) {
      if ((frame_idx++) % down_sample_rate != 0) continue;
      if (!imu_frame) continue;
      if (imu_frame->timestamp < eval_start_time ||
          imu_frame->timestamp > eval_end_time) {
        continue;
      }

      double acc_res = 0.;
      double gyr_res = 0.;
      if (!CalImuResidual(imu_label, imu_frame, acc_res, gyr_res)) {
        continue;
      }

      acc_residuals.push_back(acc_res);
      gyr_residuals.push_back(gyr_res);
      all_acc_residuals.push_back(acc_res);
      all_gyr_residuals.push_back(gyr_res);
    }

    ImuResidualStats stats;
    stats.sample_count = std::min(acc_residuals.size(), gyr_residuals.size());
    stats.acc_rmse = ComputeRmse(acc_residuals);
    stats.gyr_rmse = ComputeRmse(gyr_residuals);
    imu_residual_stats[imu_label] = stats;
  }

  const double all_acc_rmse = ComputeRmse(all_acc_residuals);
  const double all_gyr_rmse = ComputeRmse(all_gyr_residuals);

  // ===========================================================================

  // Step 3: Compute and save camera reprojection validation artifacts.
  struct CamSummary {
    int num_frames = 0;
    ReprojStats stats;
  };

  struct CamFrameValidationRecord {
    CamFrame::Ptr frame;
    std::vector<std::pair<double, double>> residuals;
    std::vector<std::pair<double, double>> observed_points;
    std::vector<std::pair<double, double>> reprojected_points;
  };

  std::map<std::string, std::vector<std::pair<double, double>>> cam_all_errors;
  std::map<std::string, std::vector<CamFrameValidationRecord>>
      cam_frame_records;
  std::map<std::string, CamSummary> cam_summaries;

  const std::string validation_root =
      (std::filesystem::path(output_path).parent_path() /
       "cam_reproj_validation")
          .string();
  std::filesystem::create_directories(validation_root);

  // Step 3.1: Evaluate all camera-frame reprojection residuals.
  const auto& cam_sequences = context_.sensor_manager->GetAllCamSequences();
  const auto& cam_configs = context_.sensor_manager->GetAllCamConfigs();
  for (const auto& [cam_label, cam_seq] : cam_sequences) {
    if (!cam_seq) {
      continue;
    }

    const int down_sample_rate =
        cam_configs.at(cam_label).down_sample_rate_ucalib;
    int frame_idx = 0;
    for (const auto& cam_frame : cam_seq->frames) {
      if ((frame_idx++) % down_sample_rate != 0) continue;
      if (!cam_frame || cam_frame->keypoints.empty()) continue;
      if (cam_frame->timestamp < eval_start_time ||
          cam_frame->timestamp > eval_end_time) {
        continue;
      }

      std::vector<std::pair<double, double>> residuals;
      std::vector<std::pair<double, double>> observed_points;
      std::vector<std::pair<double, double>> reprojected_points;
      if (!CalCamReprojResidual(cam_label, cam_frame, residuals,
                                observed_points, reprojected_points)) {
        continue;
      }

      cam_all_errors[cam_label].insert(cam_all_errors[cam_label].end(),
                                       residuals.begin(), residuals.end());
      cam_frame_records[cam_label].push_back(
          {cam_frame, residuals, observed_points, reprojected_points});
    }
  }

  // ===========================================================================

  // Step 3.2: Randomly save 10 frame-level visualizations.
  bool disable_per_frame_reproj_viz = false;
  for (const auto& [cam_label, _] : cam_configs) {
    const auto& errors_2d = cam_all_errors[cam_label];
    const ReprojStats stats = ComputeReprojStats(errors_2d);
    const int num_frames =
        static_cast<int>(cam_frame_records[cam_label].size());
    cam_summaries[cam_label] = {num_frames, stats};

    const std::string cam_dir = validation_root + "/" + cam_label;
    std::filesystem::create_directories(cam_dir);

    const std::string cam_txt =
        cam_dir + "/" + cam_label + "_all_reproj_error.txt";
    if (!SaveReprojErrorTxt(cam_txt, cam_label, num_frames, stats, errors_2d,
                            true)) {
      spdlog::warn("Failed to save unified camera reprojection txt: {}",
                   cam_txt);
    }

    const std::string cam_png =
        cam_dir + "/" + cam_label + "_all_reproj_error.png";
    if (!errors_2d.empty() &&
        !DrawReprojPlot(cam_label, errors_2d, stats, num_frames, cam_png)) {
      spdlog::warn("Failed to save unified camera reprojection plot: {}",
                   cam_png);
    }

    const std::string per_frame_dir =
        cam_dir + "/per_frame_reprojection_validation";
    std::filesystem::create_directories(per_frame_dir);
    const std::string per_frame_txt =
        per_frame_dir + "/per_frame_reproj_stats.txt";
    std::ofstream per_frame_file(per_frame_txt);
    if (!per_frame_file.is_open()) {
      spdlog::warn("Failed to save per-frame reprojection stats: {}",
                   per_frame_txt);
      continue;
    }
    per_frame_file << "# Per-frame reprojection statistics for camera: "
                   << cam_label << "\n";

    const auto& records = cam_frame_records[cam_label];
    const auto selected_indices = SelectRandomIndices(
        records.size(), std::min<size_t>(10, records.size()));
    for (const size_t idx : selected_indices) {
      const auto& record = records[idx];
      const ReprojStats frame_stats = ComputeReprojStats(record.residuals);

      std::ostringstream ts_ss;
      ts_ss << std::fixed << std::setprecision(6) << record.frame->timestamp;
      const std::string ts_str = ts_ss.str();
      const std::string filename =
          std::filesystem::path(record.frame->img_path).filename().string();

      if (!disable_per_frame_reproj_viz) {
        cv::Mat img = cv::imread(record.frame->img_path);
        if (img.empty()) {
          spdlog::warn(
              "Failed to read image for unified reprojection visualization: "
              "{}. Stop reprojection image drawing and saving, but continue "
              "statistics.",
              record.frame->img_path);
          disable_per_frame_reproj_viz = true;
        } else {
          cv::Mat viz_img = img.clone();
          const int image_size = std::min(img.rows, img.cols);
          const int reproj_radius =
              std::max(3, static_cast<int>(image_size * 0.003));
          const int line_thickness =
              std::max(1, static_cast<int>(image_size * 0.0015));
          const int obs_cross_half =
              std::max(3, static_cast<int>(image_size * 0.005));

          for (size_t i = 0; i < record.observed_points.size() &&
                             i < record.reprojected_points.size();
               ++i) {
            const cv::Point2d obs_pt(record.observed_points[i].first,
                                     record.observed_points[i].second);
            const cv::Point2d reproj_pt(record.reprojected_points[i].first,
                                        record.reprojected_points[i].second);
            cv::circle(viz_img, reproj_pt, reproj_radius, cv::Scalar(0, 0, 255),
                       -1);
            cv::line(viz_img, obs_pt, reproj_pt, cv::Scalar(0, 255, 255),
                     line_thickness);
          }

          // Draw observed crosses on top.
          for (const auto& [ox, oy] : record.observed_points) {
            const cv::Point2d obs_pt(ox, oy);
            cv::line(viz_img, cv::Point2d(obs_pt.x - obs_cross_half, obs_pt.y),
                     cv::Point2d(obs_pt.x + obs_cross_half, obs_pt.y),
                     cv::Scalar(255, 0, 0), 1);
            cv::line(viz_img, cv::Point2d(obs_pt.x, obs_pt.y - obs_cross_half),
                     cv::Point2d(obs_pt.x, obs_pt.y + obs_cross_half),
                     cv::Scalar(255, 0, 0), 1);
          }

          const std::string save_path = per_frame_dir + "/" + filename;
          if (!cv::imwrite(save_path, viz_img)) {
            spdlog::warn(
                "Failed to save unified reprojection visualization: {}",
                save_path);
          }
        }
      }

      per_frame_file << "image=" << filename << ", timestamp=" << ts_str
             << ", points=" << frame_stats.count
             << ", mean_px=" << std::fixed << std::setprecision(6)
             << frame_stats.mean
             << ", median_px=" << frame_stats.median
             << ", min_px=" << frame_stats.min
             << ", max_px=" << frame_stats.max << "\n";
    }
  }

  // ===========================================================================

  // Step 3.3: Save overall reprojection artifacts.
  std::vector<std::pair<double, double>> all_cam_errors;
  int total_cam_frames = 0;
  for (const auto& [cam_label, summary] : cam_summaries) {
    total_cam_frames += summary.num_frames;
    const auto& errors = cam_all_errors[cam_label];
    all_cam_errors.insert(all_cam_errors.end(), errors.begin(), errors.end());
  }

  const ReprojStats overall_cam_stats = ComputeReprojStats(all_cam_errors);
  const std::string overall_dir = validation_root + "/overall";
  std::filesystem::create_directories(overall_dir);

  const std::string overall_txt = overall_dir + "/overall_reproj_error.txt";
  if (!SaveReprojErrorTxt(overall_txt, "overall", total_cam_frames,
                          overall_cam_stats, all_cam_errors, false)) {
    spdlog::warn("Failed to save unified overall reprojection txt: {}",
                 overall_txt);
  }

  const std::string overall_png = overall_dir + "/overall_reproj_error.png";
  if (!all_cam_errors.empty() &&
      !DrawReprojPlot("overall", all_cam_errors, overall_cam_stats,
                      total_cam_frames, overall_png)) {
    spdlog::warn("Failed to save unified overall reprojection plot: {}",
                 overall_png);
  }

  // ===========================================================================

  // Step 3.4: Save per-camera summary used by overall report.
  const std::string cam_summary_txt =
      overall_dir + "/camera_reproj_summary.txt";
  std::ofstream cam_summary_file(cam_summary_txt);
  if (!cam_summary_file.is_open()) {
    spdlog::warn("Failed to save unified camera reprojection summary: {}",
                 cam_summary_txt);
  } else {
    cam_summary_file << "# Reprojection summary (overall + per camera)\n";
    cam_summary_file << "total_frames = " << total_cam_frames << "\n";
    cam_summary_file << "total_points = " << overall_cam_stats.count << "\n";
    cam_summary_file << "overall_mean_px = " << std::fixed
                     << std::setprecision(6) << overall_cam_stats.mean << "\n";
    cam_summary_file << "overall_median_px = " << std::fixed
                     << std::setprecision(6) << overall_cam_stats.median
                     << "\n";
    cam_summary_file << "overall_min_px = " << std::fixed
                     << std::setprecision(6) << overall_cam_stats.min << "\n";
    cam_summary_file << "overall_max_px = " << std::fixed
                     << std::setprecision(6) << overall_cam_stats.max << "\n";
    cam_summary_file << "camera_count = " << cam_summaries.size() << "\n";
    for (const auto& [cam_label, summary] : cam_summaries) {
      cam_summary_file << cam_label << ": frames=" << summary.num_frames
                       << ", points=" << summary.stats.count
                       << ", mean_px=" << std::fixed << std::setprecision(6)
                       << summary.stats.mean
                       << ", median_px=" << summary.stats.median
                       << ", min_px=" << summary.stats.min
                       << ", max_px=" << summary.stats.max << "\n";
    }
  }

  // ===========================================================================

  // Step 4: Write concise validation report.
  out_file << "# Unified calibration validation\n";
  out_file << "# Missing threshold: 1.0 s\n\n";

  out_file << "[time]\n";
  out_file << "start_sec = " << eval_start_time << "\n";
  out_file << "end_sec = " << eval_end_time << "\n";
  out_file << "duration_sec = " << total_duration << "\n\n";

  out_file << "[completeness_per_sensor]\n";
  for (const auto& [sensor_key, metrics] : completeness_map) {
    out_file << sensor_key << ": samples=" << metrics.valid_sample_count
             << ", miss_duration (s)=" << metrics.missing_duration
             << ", comp_completeness (%)=" << metrics.completeness * 100.
             << ", miss_intervals=" << metrics.missing_intervals.size() << "\n";
    for (size_t i = 0; i < metrics.missing_intervals.size(); ++i) {
      out_file << "  miss_" << i << "=[" << metrics.missing_intervals[i].first
               << "," << metrics.missing_intervals[i].second << "]\n";
    }
  }
  out_file << "\n";

  out_file << "[completeness_camera_global]\n";
  out_file << "has_camera_sensor = " << (has_camera_sensor ? "true" : "false")
           << "\n";
  out_file << "miss_duration (s) = " << camera_global_missing_duration << "\n";
  out_file << "comp_completeness (%) = " << camera_global_completeness * 100.
           << "\n";
  out_file << "miss_intervals = " << camera_global_missing_intervals.size()
           << "\n";
  for (size_t i = 0; i < camera_global_missing_intervals.size(); ++i) {
    out_file << "  miss_" << i << "=["
             << camera_global_missing_intervals[i].first << ","
             << camera_global_missing_intervals[i].second << "]\n";
  }
  out_file << "\n";

  out_file << "[imu_residual]\n";
  out_file << "solution_usable = " << (solution_usable ? "true" : "false")
           << "\n";
  for (const auto& [imu_label, stats] : imu_residual_stats) {
    out_file << imu_label << ": samples=" << stats.sample_count
             << ", acc_rmse (m/s^2)=" << stats.acc_rmse
             << ", gyr_rmse (rad/s)=" << stats.gyr_rmse << "\n";
  }
  out_file << "all_acc_rmse (m/s^2) = " << all_acc_rmse << "\n";
  out_file << "all_gyr_rmse (rad/s) = " << all_gyr_rmse << "\n";

  out_file << "\n[camera_reproj_summary]\n";
  out_file << "total_frames = " << total_cam_frames << "\n";
  out_file << "total_points = " << overall_cam_stats.count << "\n";
  out_file << "overall_mean_px = " << overall_cam_stats.mean << "\n";
  out_file << "overall_median_px = " << overall_cam_stats.median << "\n";
  out_file << "overall_min_px = " << overall_cam_stats.min << "\n";
  out_file << "overall_max_px = " << overall_cam_stats.max << "\n";
  out_file << "camera_count = " << cam_summaries.size() << "\n";
  for (const auto& [cam_label, summary] : cam_summaries) {
    out_file << cam_label << ": frames=" << summary.num_frames
             << ", points=" << summary.stats.count
             << ", mean_px=" << summary.stats.mean
             << ", median_px=" << summary.stats.median
             << ", min_px=" << summary.stats.min
             << ", max_px=" << summary.stats.max << "\n";
  }

  // ===========================================================================

  return true;
}

bool CalibrationValidator::CalMetaMinMaxTime(const double& meas_time,
                                             double& min_time,
                                             double& max_time) {
  // Obtain the maximum and minimum timestamps of metadata based on the
  // maximum change in time offset, and ensure their validity.
  const double kMaxToffChange =
      context_.system_config->unified_calib_config.max_toff_change;

  double spline_start_time = context_.trans_spline_info.start_time;
  double spline_end_time = context_.trans_spline_info.end_time;
  if ((meas_time < spline_start_time - 2 * kMaxToffChange) ||
      (meas_time > spline_end_time + 2 * kMaxToffChange)) {
    spdlog::critical(
        "The specified measurement time is not within the valid time range "
        "of the system. ");
    return false;
  }

  if (meas_time < spline_start_time) {
    min_time = spline_start_time;
    max_time = spline_start_time + kMaxToffChange;
  } else if (meas_time - kMaxToffChange < spline_start_time &&
             meas_time >= spline_start_time) {
    min_time = spline_start_time;
    max_time = meas_time + kMaxToffChange;
  } else if (meas_time > spline_end_time) {
    min_time = spline_end_time - kMaxToffChange;
    max_time = spline_end_time;
  } else if (meas_time + kMaxToffChange > spline_end_time &&
             meas_time <= spline_end_time) {
    min_time = meas_time - kMaxToffChange;
    max_time = spline_end_time;
  } else {
    min_time = meas_time - kMaxToffChange;
    max_time = meas_time + kMaxToffChange;
  }

  return true;
}

}  // namespace xr_ucalib

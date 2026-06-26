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
#include "xr_ucalib/uc_cam_calib/sfm_calib/handeye_calibrator.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <sophus/se3.hpp>
#include <spdlog/spdlog.h>
// clang-format on

namespace xr_ucalib {

namespace {

double ComputeMedian(std::vector<double> values) {
  if (values.empty()) return 0.0;
  size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + mid, values.end());
  double median = values[mid];
  if (values.size() % 2 == 0) {
    auto max_it = std::max_element(values.begin(), values.begin() + mid);
    median = 0.5 * (median + *max_it);
  }
  return median;
}

double ComputeMad(const std::vector<double>& values, double median) {
  if (values.empty()) return 0.0;
  std::vector<double> abs_dev;
  abs_dev.reserve(values.size());
  for (double v : values) {
    abs_dev.push_back(std::abs(v - median));
  }
  return ComputeMedian(abs_dev);
}

/**
 * @brief Interpolate pose at timestamp t.
 *
 * @param[in] poses Input poses sorted by timestamp.
 * @param[in] t Target timestamp.
 * @param[out] out_pose Output interpolated pose.
 * @return true If successful.
 */
bool InterpolatePose(const std::vector<StampedPose>& poses, double t,
                     StampedPose& out_pose) {
  if (poses.empty()) return false;
  if (t < poses.front().timestamp || t > poses.back().timestamp) return false;

  auto it = std::lower_bound(
      poses.begin(), poses.end(), t,
      [](const StampedPose& p, double val) { return p.timestamp < val; });

  if (it == poses.begin()) {
    out_pose = *it;
    return std::abs(it->timestamp - t) < 1e-6;
  }

  auto prev_it = std::prev(it);
  double t1 = prev_it->timestamp;
  double t2 = it->timestamp;
  double alpha = (t - t1) / (t2 - t1);

  out_pose.timestamp = t;
  out_pose.trans = (1.0 - alpha) * prev_it->trans + alpha * it->trans;
  out_pose.rot_q = prev_it->rot_q.slerp(alpha, it->rot_q);

  return true;
}

}  // namespace

bool HandEyeCalibrator::SolveBodyExtrinsics(
    const std::vector<StampedPose>& poses_ref,
    const std::vector<StampedPose>& poses_tgt, Eigen::Matrix4d& T_ref_tgt) {
  constexpr size_t kMinPoses = 6;
  if (poses_ref.size() < kMinPoses || poses_tgt.size() < kMinPoses) {
    spdlog::error("Not enough poses for hand-eye calibration.");
    return false;
  }

  // Step 1: Synchronize poses.
  double start_time =
      std::max(poses_ref.front().timestamp, poses_tgt.front().timestamp);
  double end_time =
      std::min(poses_ref.back().timestamp, poses_tgt.back().timestamp);

  std::vector<StampedPose> sync_ref, sync_tgt;

  for (const auto& p_ref : poses_ref) {
    if (p_ref.timestamp < start_time) continue;
    if (p_ref.timestamp > end_time) break;

    StampedPose p_tgt;
    if (InterpolatePose(poses_tgt, p_ref.timestamp, p_tgt)) {
      sync_ref.push_back(p_ref);
      sync_tgt.push_back(p_tgt);
    }
  }

  if (sync_ref.size() < kMinPoses) {
    spdlog::error(
        "Not enough synchronized poses ({}) for hand-eye calibration. Please "
        "ensure that the clock sources of the calibration sequences are "
        "synchronized.",
        sync_ref.size());
    return false;
  }

  // ===========================================================================

  // Step 2: Compute relative motions using consecutive frames.
  // This approach uses adjacent pose pairs and automatically filters out
  // small rotations through the threshold below, eliminating the need for
  // a hardcoded step size.
  std::vector<Sophus::SE3d> A_vec, B_vec;
  constexpr double kMinRotationNorm = 0.05;  // ~2.8 degrees, to avoid noise
  size_t total_frame_pairs = 0;

  for (size_t i = 0; i < sync_ref.size() - 1; ++i) {
    size_t j = i + 1;
    total_frame_pairs++;

    Sophus::SE3d T_ref_i(sync_ref[i].rot_q, sync_ref[i].trans);
    Sophus::SE3d T_ref_j(sync_ref[j].rot_q, sync_ref[j].trans);
    Sophus::SE3d A = T_ref_i.inverse() * T_ref_j;

    Sophus::SE3d T_tgt_i(sync_tgt[i].rot_q, sync_tgt[i].trans);
    Sophus::SE3d T_tgt_j(sync_tgt[j].rot_q, sync_tgt[j].trans);
    Sophus::SE3d B = T_tgt_i.inverse() * T_tgt_j;

    // Filter small rotations to avoid noise
    if (A.so3().log().norm() < kMinRotationNorm) continue;

    A_vec.push_back(A);
    B_vec.push_back(B);
  }

  if (A_vec.size() < kMinPoses / 2) {
    spdlog::error(
        "Insufficient rotation excitation: only {} valid motion pairs found "
        "(threshold: {}).",
        A_vec.size(), kMinPoses / 2);
    return false;
  }

  // ===========================================================================

  // Step 3: Solve AX=XB using linear method.
  // Step 3.1: Solve for Rotation.
  // We have r_A = R_X * r_B where r is the rotation vector (axis * angle).
  std::vector<Eigen::Vector3d> r_A_vec, r_B_vec;
  r_A_vec.reserve(A_vec.size());
  r_B_vec.reserve(A_vec.size());

  for (size_t i = 0; i < A_vec.size(); ++i) {
    r_A_vec.push_back(A_vec[i].so3().log());
    r_B_vec.push_back(B_vec[i].so3().log());
  }

  auto SolveRotation = [](const std::vector<Eigen::Vector3d>& r_A,
                          const std::vector<Eigen::Vector3d>& r_B) {
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < r_A.size(); ++i) {
      H += r_B[i] * r_A[i].transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    Eigen::Matrix3d R = V * U.transpose();
    if (R.determinant() < 0) {
      V.col(2) *= -1;
      R = V * U.transpose();
    }
    return R;
  };

  Eigen::Matrix3d R_X = SolveRotation(r_A_vec, r_B_vec);

  // 3.2 Solve for translation.
  // (R_A - I) t_X = R_X t_B - t_A
  // Use extended form: (R_A - I) t_X + s * t_A = R_X * t_B to handle scale
  // drift or numerical issues.
  auto SolveTranslation = [&](const std::vector<Sophus::SE3d>& A_list,
                              const std::vector<Sophus::SE3d>& B_list,
                              const Eigen::Matrix3d& R_X_in,
                              Eigen::Vector3d& t_X_out, double& scale_out) {
    Eigen::MatrixXd trans_solver_N =
        Eigen::MatrixXd::Zero(3 * A_list.size(), 4);
    Eigen::VectorXd trans_solver_b = Eigen::VectorXd::Zero(3 * A_list.size());

    for (size_t i = 0; i < A_list.size(); ++i) {
      Eigen::Matrix3d R_A = A_list[i].rotationMatrix();
      Eigen::Vector3d t_A = A_list[i].translation();
      Eigen::Vector3d t_B = B_list[i].translation();

      // Fill N: [R_A - I, t_A]
      trans_solver_N.block<3, 3>(3 * i, 0) = R_A - Eigen::Matrix3d::Identity();
      trans_solver_N.block<3, 1>(3 * i, 3) = t_A;
      // Fill b: R_X * t_B
      trans_solver_b.segment<3>(3 * i) = R_X_in * t_B;
    }

    Eigen::Vector4d trans_solver_result =
        trans_solver_N.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
            .solve(trans_solver_b);

    scale_out = trans_solver_result(3);
    if (std::abs(scale_out) > 1e-6) {
      trans_solver_result /= scale_out;
    }
    t_X_out = trans_solver_result.head<3>();
  };

  Eigen::Vector3d t_X = Eigen::Vector3d::Zero();
  double scale_factor = 1.0;
  SolveTranslation(A_vec, B_vec, R_X, t_X, scale_factor);

  if (std::abs(scale_factor) <= 1e-6) {
    spdlog::warn(
        "Computed scale factor close to zero in hand-eye calibration "
        "translation solver.");
  }

  T_ref_tgt = Eigen::Matrix4d::Identity();
  T_ref_tgt.block<3, 3>(0, 0) = R_X;
  T_ref_tgt.block<3, 1>(0, 3) = t_X;

  // ===========================================================================

  // Step 4: Quality check + robust refinement
  constexpr double kRotErrThresh = 0.1;     // radians
  constexpr double kTransErrThresh = 0.05;  // meters

  // Helper function to compute residuals for AX=XB constraint
  auto ComputeResiduals =
      [&](const std::vector<Sophus::SE3d>& A_list,
          const std::vector<Sophus::SE3d>& B_list, const Sophus::SE3d& X_in,
          std::vector<double>& rot_errs, std::vector<double>& trans_errs) {
        rot_errs.clear();
        trans_errs.clear();
        rot_errs.reserve(A_list.size());
        trans_errs.reserve(A_list.size());
        for (size_t i = 0; i < A_list.size(); ++i) {
          Sophus::SE3d LHS = A_list[i] * X_in;
          Sophus::SE3d RHS = X_in * B_list[i];
          Sophus::SE3d Err = LHS.inverse() * RHS;
          rot_errs.push_back(Err.so3().log().norm());
          trans_errs.push_back(Err.translation().norm());
        }
      };

  // Step 4.1: Compute residuals and identify outliers
  Sophus::SE3d X(T_ref_tgt);
  std::vector<double> rot_errs, trans_errs;
  ComputeResiduals(A_vec, B_vec, X, rot_errs, trans_errs);

  double rot_med = ComputeMedian(rot_errs);
  double rot_mad = ComputeMad(rot_errs, rot_med);
  double trans_med = ComputeMedian(trans_errs);
  double trans_mad = ComputeMad(trans_errs, trans_med);

  double rot_thresh = std::max(kRotErrThresh, 3.0 * rot_mad + 1e-6);
  double trans_thresh = std::max(kTransErrThresh, 3.0 * trans_mad + 1e-6);

  std::vector<Sophus::SE3d> A_inliers, B_inliers;
  A_inliers.reserve(A_vec.size());
  B_inliers.reserve(B_vec.size());
  for (size_t i = 0; i < A_vec.size(); ++i) {
    if (rot_errs[i] <= rot_thresh && trans_errs[i] <= trans_thresh) {
      A_inliers.push_back(A_vec[i]);
      B_inliers.push_back(B_vec[i]);
    }
  }

  // Step 4.2: Re-optimize on inliers if outliers are detected
  if (A_inliers.size() >= kMinPoses / 2 && A_inliers.size() < A_vec.size()) {
    std::vector<Eigen::Vector3d> r_A_in, r_B_in;
    r_A_in.reserve(A_inliers.size());
    r_B_in.reserve(B_inliers.size());
    for (size_t i = 0; i < A_inliers.size(); ++i) {
      r_A_in.push_back(A_inliers[i].so3().log());
      r_B_in.push_back(B_inliers[i].so3().log());
    }

    R_X = SolveRotation(r_A_in, r_B_in);
    SolveTranslation(A_inliers, B_inliers, R_X, t_X, scale_factor);

    T_ref_tgt = Eigen::Matrix4d::Identity();
    T_ref_tgt.block<3, 3>(0, 0) = R_X;
    T_ref_tgt.block<3, 1>(0, 3) = t_X;
    X = Sophus::SE3d(T_ref_tgt);

    ComputeResiduals(A_inliers, B_inliers, X, rot_errs, trans_errs);
  }

  // Step 4.3: Compute final error statistics and log results
  double total_rot_err = 0.0;
  double total_trans_err = 0.0;
  for (size_t i = 0; i < rot_errs.size(); ++i) {
    total_rot_err += rot_errs[i];
    total_trans_err += trans_errs[i];
  }

  double mean_rot_err = total_rot_err / rot_errs.size();
  double mean_trans_err = total_trans_err / trans_errs.size();

  if (mean_rot_err > kRotErrThresh) {
    spdlog::warn("High rotation error in hand-eye calibration: {:.3f} rad",
                 mean_rot_err);
  }
  if (mean_trans_err > kTransErrThresh) {
    spdlog::warn("High translation error in hand-eye calibration: {:.3f} m",
                 mean_trans_err);
  }

  // ===========================================================================

  return true;
}

bool HandEyeCalibrator::AlignWorldFrames(
    const std::vector<StampedPose>& poses_ref,
    const std::vector<StampedPose>& poses_tgt, Eigen::Matrix4d& T_ref_tgt) {
  constexpr size_t kMinPoses = 6;
  if (poses_ref.size() < kMinPoses || poses_tgt.size() < kMinPoses) {
    spdlog::error("Not enough poses for hand-eye calibration.");
    return false;
  }

  // Step 1: Synchronize poses.
  double start_time =
      std::max(poses_ref.front().timestamp, poses_tgt.front().timestamp);
  double end_time =
      std::min(poses_ref.back().timestamp, poses_tgt.back().timestamp);

  std::vector<StampedPose> sync_ref, sync_tgt;

  for (const auto& p_ref : poses_ref) {
    if (p_ref.timestamp < start_time) continue;
    if (p_ref.timestamp > end_time) break;

    StampedPose p_tgt;
    if (InterpolatePose(poses_tgt, p_ref.timestamp, p_tgt)) {
      sync_ref.push_back(p_ref);
      sync_tgt.push_back(p_tgt);
    }
  }

  if (sync_ref.size() < 3) {
    spdlog::error("Not enough synchronized poses ({}) for world alignment.",
                  sync_ref.size());
    return false;
  }

  // ===========================================================================

  // Step 2: Collect point pairs for Umeyama.
  // We align the trajectories directly, assuming they represent the same body.
  std::vector<Eigen::Vector3d> pts_in_ref;
  std::vector<Eigen::Vector3d> pts_in_tgt;

  for (size_t i = 0; i < sync_ref.size(); ++i) {
    // Position of body expressed in W1
    Eigen::Vector3d p_ref = sync_ref[i].trans;

    // Position of body expressed in W2
    Eigen::Vector3d p_tgt = sync_tgt[i].trans;

    pts_in_ref.push_back(p_ref);
    pts_in_tgt.push_back(p_tgt);
  }

  // ===========================================================================

  // Step 3: Compute Umeyama alignment.
  Eigen::Matrix3Xd src(3, pts_in_tgt.size());
  Eigen::Matrix3Xd dst(3, pts_in_ref.size());

  for (size_t i = 0; i < pts_in_tgt.size(); ++i) {
    src.col(i) = pts_in_tgt[i];
    dst.col(i) = pts_in_ref[i];
  }

  // T_ref_tgt * src = dst
  T_ref_tgt = Eigen::umeyama(src, dst, false);  // false = no scaling

  // ===========================================================================

  // Step 4: Quality check.
  constexpr double kErrThresh = 0.05;  // meters
  double sq_err_sum = 0.0;

  Sophus::SE3d T(T_ref_tgt);
  for (size_t i = 0; i < pts_in_ref.size(); ++i) {
    Eigen::Vector3d p1 = pts_in_ref[i];
    Eigen::Vector3d p2_transformed = T * pts_in_tgt[i];
    sq_err_sum += (p1 - p2_transformed).squaredNorm();
  }
  double rmse = std::sqrt(sq_err_sum / pts_in_ref.size());

  if (rmse > kErrThresh) {
    spdlog::warn("High RMSE in World Alignment: {:.3f}", rmse);
  }

  // ===========================================================================

  return true;
}

}  // namespace xr_ucalib
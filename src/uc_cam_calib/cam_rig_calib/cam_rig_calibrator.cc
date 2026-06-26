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
#include "xr_ucalib/uc_cam_calib/cam_rig_calib/cam_rig_calibrator.h" 

#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

#include "xr_ucalib/uc_cam_calib/cam_rig_calib/cam_reproj_cost.hpp"
// clang-format on

namespace xr_ucalib {

bool CamRigCalibrator::RunCalibration() {
  spdlog::info("---------------- Camera Rig Calibration ---------------");

  // We require that SFM calibration has been completed before camera rig
  // calibration.
  uint8_t required_status = CalibParameters::ParamStatus::SETUP |
                            CalibParameters::ParamStatus::SFM_CALIB;
  if ((calib_parameters_->param_status & required_status) != required_status) {
    spdlog::error(
        "SFM calibration must be completed before camera rig calibration. "
        "Current param_status: {}",
        static_cast<int>(calib_parameters_->param_status));
    return false;
  }

  // ===========================================================================

  // Step 1: Build camera frame correspondences.
  // Correspondence frames of all cameras. Key: base camera frame index, Value:
  // map of (camera label -> frame pointer).
  FrameCorrespondence frame_correspondence;
  if (!BuildCamFrameCorrespondences(frame_correspondence)) {
    spdlog::error("Failed to build camera frame correspondences.");
    return false;
  }
  spdlog::info("Built {} camera frame correspondences (after down-sampling).",
               frame_correspondence.size());

  // ===========================================================================

  // Step 2: Estimate initial camera poses for each frame correspondence.
  // Initial poses of the base camera (Transform from base camera to world).
  CamPoses cam_poses_W_Cb;
  if (!InitializeCamPoses(frame_correspondence, cam_poses_W_Cb)) {
    spdlog::error("Failed to initialize camera poses for camera rig.");
    return false;
  }
  spdlog::info("Initialized camera poses for {} frames.",
               cam_poses_W_Cb.size());

  // ===========================================================================

  // Step 3: Build and optimize the Ceres problem (Bundle Adjustment).
  if (!BuildAndOptimizeCeresProblem(frame_correspondence, cam_poses_W_Cb)) {
    spdlog::error("Failed to optimize camera rig calibration problem.");
    return false;
  }
  spdlog::info("Camera rig calibration completed.");

  // Outlier rejection and re-optimization.
  // Check if any camera has outlier rejection enabled (reproj_threshold >= 0).
  bool outlier_rejection_enabled = false;
  for (const auto& [cam_label, cam_config] :
       sensor_manager_->GetAllCamConfigs()) {
    if (cam_config.reproj_threshold >= 0) {
      outlier_rejection_enabled = true;
      break;
    }
  }

  // If outlier rejection is enabled, remove outliers and re-optimize.
  if (outlier_rejection_enabled) {
    size_t num_outliers = reproj_evaluator_->RemoveReprojOutliers(
        cam_poses_W_Cb, frame_correspondence);

    if (num_outliers > 0) {
      spdlog::info("Removed {} outlier keypoints. Re-running optimization.",
                   num_outliers);

      if (!BuildAndOptimizeCeresProblem(frame_correspondence, cam_poses_W_Cb)) {
        spdlog::error(
            "Failed to optimize camera rig calibration after outlier "
            "removal.");
        return false;
      }
      spdlog::info("Re-optimization after outlier removal completed.");
    } else {
      spdlog::info("No outliers found, skipping re-optimization.");
    }
  }

  // ===========================================================================

  // Step 4: Compute and evaluate reprojection errors.
  if (system_config_->workspace_dir == "") {
    spdlog::error(
        "Workspace directory is not set in the system configuration.");
    return false;
  }
  std::string work_dir = system_config_->workspace_dir + "/cam_rig_calib";
  if (!std::filesystem::exists(work_dir)) {
    std::filesystem::create_directories(work_dir);
  }

  spdlog::info("Evaluating reprojection errors...");
  ReprojEvaluator::ReprojErrorPerFrame reproj_errors;
  if (!reproj_evaluator_->ComputeReprojError(
          cam_poses_W_Cb, frame_correspondence, reproj_errors)) {
    spdlog::warn("Failed to compute reprojection errors.");
  }

  if (!reproj_evaluator_->EvaluateReprojError(reproj_errors, work_dir)) {
    spdlog::warn("Failed to evaluate reprojection errors.");
  }

  // Save per-frame reprojection validation images.
  // If save_all_reproj_image is false, the evaluator will save a uniform
  // sample of frames.
  if (!reproj_evaluator_->SaveReprojImage(cam_poses_W_Cb, frame_correspondence,
                                          work_dir)) {
    spdlog::warn("Failed to visualize reprojection.");
  }

  // ===========================================================================

  spdlog::info("------------ Camera rig calibration results -----------");

  // Print the results.
  PrintCalibrationResults();

  calib_parameters_->param_status |=
      CalibParameters::ParamStatus::CAM_RIG_CALIB;

  return true;
}

bool CamRigCalibrator::BuildCamFrameCorrespondences(
    FrameCorrespondence& frame_correspondence) {
  // Step 1: Validate inputs.
  const auto& all_cam_sequences = sensor_manager_->GetAllCamSequences();
  if (all_cam_sequences.empty()) {
    spdlog::error("No camera sequences found in sensor manager.");
    return false;
  }
  const std::string& base_cam_label = calib_parameters_->base_camera_label;
  if (base_cam_label.empty()) {
    spdlog::error("Base camera label is not set in calibration parameters.");
    return false;
  }
  auto base_cam_iter = all_cam_sequences.find(base_cam_label);
  if (base_cam_iter == all_cam_sequences.end()) {
    spdlog::error("Base camera '{}' not found in sensor manager.",
                  base_cam_label);
    return false;
  }
  const CamSequence::Ptr& base_cam_seq = base_cam_iter->second;
  if (base_cam_seq->Empty()) {
    spdlog::error("Base camera '{}' has no frames.", base_cam_label);
    return false;
  }

  // ===========================================================================

  // Step 2: Build camera frame correspondences according to timestamps.
  // Camera frame correspondences Key: base camera frame index, Value: map of
  // (camera label -> corresponding frame pointer).
  FrameCorrespondence frame_corr;

  // Initialize with base camera frames.
  for (size_t i = 0; i < base_cam_seq->Size(); ++i) {
    frame_corr[static_cast<int>(i)][base_cam_label] = base_cam_seq->At(i);
  }

  // Maximum timestamp difference for frame correspondence (100 us).
  constexpr double kMaxTimeDiff = 1e-4;
  // Find correspondences for other cameras.
  for (const auto& [cam_label, cam_seq] : all_cam_sequences) {
    if (cam_label == base_cam_label) {
      continue;
    }

    if (cam_seq->Empty()) {
      spdlog::warn("Camera '{}' has no frames, skipping.", cam_label);
      continue;
    }

    // Since timestamps are monotonically increasing, we can use a two-pointer
    // approach.
    size_t other_idx = 0;
    for (size_t base_idx = 0; base_idx < base_cam_seq->Size(); ++base_idx) {
      const double base_timestamp = base_cam_seq->At(base_idx)->timestamp;

      while (other_idx < cam_seq->Size() && cam_seq->At(other_idx)->timestamp <
                                                base_timestamp - kMaxTimeDiff) {
        ++other_idx;
      }

      if (other_idx >= cam_seq->Size()) break;

      const double other_timestamp = cam_seq->At(other_idx)->timestamp;
      const double time_diff = std::abs(other_timestamp - base_timestamp);

      if (time_diff <= kMaxTimeDiff) {
        frame_corr[static_cast<int>(base_idx)][cam_label] =
            cam_seq->At(other_idx);
      }
    }
  }

  // ===========================================================================

  // Step 3: Check match ratio - count frames where all cameras have
  // correspondence.
  // In our discrete-time camera rig calibration, we require all cameras to have
  // corresponding frames for a valid calibration frame.
  const size_t num_cameras = all_cam_sequences.size();
  int fully_matched_count = 0;
  for (const auto& [base_idx, correspondence] : frame_corr) {
    if (correspondence.size() == num_cameras) {
      ++fully_matched_count;
    }
  }

  const double match_ratio =
      static_cast<double>(fully_matched_count) / base_cam_seq->Size();
  if (match_ratio < 0.5) {
    spdlog::error(
        "Insufficient frame correspondences ({:.1f}%). Please ensure all "
        "cameras are hardware-synchronized and have good visibility of the "
        "visual target.",
        match_ratio * 100.0);
    return false;
  }

  // ===========================================================================

  // Step 4: Down-sample frame correspondences based on configuration, and
  // filter frames without enough keypoints.
  frame_correspondence.clear();

  const int kDownSampleRate =
      system_config_->cam_calib_config.cam_down_sample_rate;
  constexpr int kMinKeypoints = 20;

  size_t skipped_no_keypoints = 0;
  int output_frame_id = 0;
  size_t base_idx = 0;
  while (base_idx < base_cam_seq->Size()) {
    const auto& correspondence = frame_corr[static_cast<int>(base_idx)];

    if (correspondence.size() != num_cameras) {
      ++base_idx;
      continue;
    }

    // Check if at least one camera has enough keypoints.
    bool any_has_enough_keypoints = false;
    for (const auto& [cam_label, frame_ptr] : correspondence) {
      if (frame_ptr->keypoints.size() >= static_cast<size_t>(kMinKeypoints)) {
        any_has_enough_keypoints = true;
        break;
      }
    }

    if (!any_has_enough_keypoints) {
      ++skipped_no_keypoints;
      ++base_idx;
      continue;
    }

    frame_correspondence[output_frame_id] = correspondence;

    ++output_frame_id;
    base_idx += kDownSampleRate;
  }

  if (output_frame_id == 0) {
    spdlog::error("No valid frames after down-sampling and filtering.");
    return false;
  }

  if (skipped_no_keypoints > base_cam_seq->Size() / 10.) {
    spdlog::error(
        "More than 10% of frames are skipped due to insufficient keypoints "
        "(minimum required: {}}). Please ensure good visibility of the "
        "fiducial "
        "target in all camera views.",
        kMinKeypoints);

    return false;
  }

  // ===========================================================================

  return true;
}

bool CamRigCalibrator::InitializeCamPoses(
    const FrameCorrespondence& frame_correspondence, CamPoses& cam_poses_W_Cb) {
  if (frame_correspondence.empty()) {
    spdlog::error(
        "Frame correspondence is empty, cannot initialize camera poses.");
    return false;
  }

  // Step 1: Prepare 3D target corners in world frame.
  auto& target_corners_3d = sensor_manager_->GetTargetCorners()->corners;
  for (auto& [_, corner] : target_corners_3d) {
    Eigen::Vector3d p_in_Ti = corner.position_local;

    int target_idx = corner.target_idx;
    Eigen::Vector3d trans_W_Ti = calib_parameters_->trans_W_Ti.at(target_idx);
    Sophus::SO3d rot_W_Ti = calib_parameters_->rot_W_Ti.at(target_idx);
    Eigen::Vector3d p_in_W = rot_W_Ti * p_in_Ti + trans_W_Ti;

    corner.position_global = p_in_W;
  }

  // ===========================================================================

  // Step 2: For each frame correspondence, solve PnP to get initial camera
  // pose. (Prepare task iterators.)
  std::vector<FrameCorrespondence::const_iterator> task_iters;
  task_iters.reserve(frame_correspondence.size());
  for (auto it = frame_correspondence.begin(); it != frame_correspondence.end();
       ++it) {
    task_iters.push_back(it);
  }

  std::atomic<int> valid_pose_count(0);
  std::atomic<int> processed_count(0);
  std::mutex cam_poses_mutex;
  std::mutex io_mutex;
  const size_t total_tasks = task_iters.size();
  const int kMultiThreadNum =
      std::max(1, system_config_->cam_calib_config.multi_thread_num);

  auto worker_func = [&](int start_idx, int end_idx) {
    for (int idx = start_idx; idx < end_idx; ++idx) {
      const auto& [base_idx, cam_map] = *task_iters[idx];
      std::string best_cam_label;
      size_t max_matches = 0;

      // Step 2.1: Find the camera with the most keypoints and matches with 3D
      // target corners.
      for (const auto& [cam_label, frame_ptr] : cam_map) {
        if (frame_ptr->keypoints.size() > max_matches) {
          max_matches = frame_ptr->keypoints.size();
          best_cam_label = cam_label;
        }
      }

      std::vector<cv::Point3f> obj_pts;
      std::vector<cv::Point2f> img_pts;
      if (cam_map.find(best_cam_label) == cam_map.end()) continue;
      const auto& frame_ptr = cam_map.at(best_cam_label);

      // Match keypoints with 3D corners.
      for (const auto& [id, kp] : frame_ptr->keypoints) {
        if (target_corners_3d.count(id)) {
          Eigen::Vector3d p_w = target_corners_3d.at(id).position_global;
          obj_pts.emplace_back(p_w.x(), p_w.y(), p_w.z());
          img_pts.emplace_back(kp.x(), kp.y());
        }
      }

      // ===========================================================================

      // Step 2.2: Undistort image points using the camera intrinsics.
      auto cam_intrinsic = calib_parameters_->cam_intrinsics.at(best_cam_label);
      cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
      K.at<double>(0, 0) = cam_intrinsic->parameters[0];  // fx
      K.at<double>(1, 1) = cam_intrinsic->parameters[1];  // fy
      K.at<double>(0, 2) = cam_intrinsic->parameters[2];  // cx
      K.at<double>(1, 2) = cam_intrinsic->parameters[3];  // cy

      // Undistort image points.
      cv::Mat D;
      std::vector<cv::Point2f> undistorted_pts;

      if (cam_intrinsic->cam_model_type == CamModelType::RADTAN) {
        // k1, k2, p1, p2
        D = (cv::Mat_<double>(4, 1) << cam_intrinsic->parameters[4],
             cam_intrinsic->parameters[5], cam_intrinsic->parameters[6],
             cam_intrinsic->parameters[7]);
        cv::undistortPoints(img_pts, undistorted_pts, K, D);
      } else if (cam_intrinsic->cam_model_type == CamModelType::EQUIDISTANT) {
        // k1, k2, k3, k4
        D = (cv::Mat_<double>(4, 1) << cam_intrinsic->parameters[4],
             cam_intrinsic->parameters[5], cam_intrinsic->parameters[6],
             cam_intrinsic->parameters[7]);
        cv::fisheye::undistortPoints(img_pts, undistorted_pts, K, D);
      } else {
        std::lock_guard<std::mutex> lock(io_mutex);
        std::printf("\n");
        spdlog::warn("Unsupported camera model type for PnP initialization: {}",
                     static_cast<int>(cam_intrinsic->cam_model_type));
        continue;
      }

      // Sanity check on undistorted points: for fisheye lenses, undistortPoints
      // outputs pinhole-normalized coords (X/Z, Y/Z). At wide angles (>~75°),
      // these values blow up, causing PnP to fail. Filter out such unreliable
      // points.
      {
        constexpr float kMaxNormalizedCoord = 3.0f;  // ~71° from optical axis
        std::vector<cv::Point3f> filtered_obj;
        std::vector<cv::Point2f> filtered_img;
        filtered_obj.reserve(obj_pts.size());
        filtered_img.reserve(undistorted_pts.size());
        for (size_t i = 0; i < undistorted_pts.size(); ++i) {
          if (std::abs(undistorted_pts[i].x) < kMaxNormalizedCoord &&
              std::abs(undistorted_pts[i].y) < kMaxNormalizedCoord &&
              std::isfinite(undistorted_pts[i].x) &&
              std::isfinite(undistorted_pts[i].y)) {
            filtered_obj.push_back(obj_pts[i]);
            filtered_img.push_back(undistorted_pts[i]);
          }
        }
        obj_pts = std::move(filtered_obj);
        undistorted_pts = std::move(filtered_img);
      }

      // ===========================================================================

      // Step 2.3: Solve PnP with identity K and zero D on normalized points.
      cv::Mat K_eye = cv::Mat::eye(3, 3, CV_64F);
      cv::Mat D_zero = cv::Mat::zeros(4, 1, CV_64F);
      cv::Mat r_vec, t_vec;

      // Minimum number of points for reliable PnP.
      constexpr size_t kMinPnPPoints = 6;
      if (obj_pts.size() < kMinPnPPoints) {
        std::lock_guard<std::mutex> lock(io_mutex);
        std::printf("\n");
        spdlog::warn("Too few matched points ({}) for frame {}, skipping.",
                     obj_pts.size(), base_idx);
        continue;
      }

      // Use RANSAC-based PnP for robustness against outliers and noise.
      cv::Mat inlier_mask;
      constexpr double kRansacReprojThreshold = 0.1;
      constexpr int kRansacIterations = 300;
      constexpr double kRansacConfidence = 0.99;

      bool pnp_success = cv::solvePnPRansac(
          obj_pts, undistorted_pts, K_eye, D_zero, r_vec, t_vec, false,
          kRansacIterations, kRansacReprojThreshold, kRansacConfidence,
          inlier_mask, cv::SOLVEPNP_EPNP);

      if (pnp_success) {
        // Collect inlier points for refinement.
        std::vector<cv::Point3f> inlier_obj_pts;
        std::vector<cv::Point2f> inlier_img_pts;
        for (int i = 0; i < inlier_mask.rows; ++i) {
          if (inlier_mask.at<int>(i)) {
            inlier_obj_pts.push_back(obj_pts[i]);
            inlier_img_pts.push_back(undistorted_pts[i]);
          }
        }

        // Check inlier ratio - if too low, result is unreliable.
        double inlier_ratio =
            static_cast<double>(inlier_obj_pts.size()) / obj_pts.size();
        if (inlier_ratio < 0.3 || inlier_obj_pts.size() < kMinPnPPoints) {
          std::lock_guard<std::mutex> lock(io_mutex);
          std::printf("\n");
          spdlog::warn(
              "PnP RANSAC for frame {} has low inlier ratio: {:.1f}% "
              "({}/{}), skipping.",
              base_idx, inlier_ratio * 100.0, inlier_obj_pts.size(),
              obj_pts.size());
          continue;
        }

        // Refine with inliers only using iterative method.
        cv::solvePnP(inlier_obj_pts, inlier_img_pts, K_eye, D_zero, r_vec,
                     t_vec, true, cv::SOLVEPNP_ITERATIVE);

        // Update obj_pts and undistorted_pts to inliers for later validation.
        obj_pts = std::move(inlier_obj_pts);
        undistorted_pts = std::move(inlier_img_pts);
      } else {
        std::lock_guard<std::mutex> lock(io_mutex);
        std::printf("\n");
        spdlog::warn("PnP RANSAC failed for frame {}.", base_idx);
        continue;
      }

      // ===========================================================================

      // Step 2.4: Validate PnP result before using it.
      // Check 1: Verify that the translation is reasonable (not too far).
      constexpr double kMaxTranslationNorm = 100.0;  // meters
      double trans_norm = cv::norm(t_vec);
      if (trans_norm > kMaxTranslationNorm) {
        std::lock_guard<std::mutex> lock(io_mutex);
        std::printf("\n");
        spdlog::warn(
            "PnP result for frame {} has unreasonable translation "
            "norm: {:.2f}m, skipping.",
            base_idx, trans_norm);
        continue;
      }

      // Check 2: Verify that all 3D points have positive depth in camera frame.
      cv::Mat R_cv_check;
      cv::Rodrigues(r_vec, R_cv_check);
      bool all_points_in_front = true;
      for (const auto& pt : obj_pts) {
        // Transform point to camera frame: p_cam = R * p_world + t
        double z_cam = R_cv_check.at<double>(2, 0) * pt.x +
                       R_cv_check.at<double>(2, 1) * pt.y +
                       R_cv_check.at<double>(2, 2) * pt.z + t_vec.at<double>(2);
        if (z_cam <= 0.01) {  // Point behind camera or too close
          all_points_in_front = false;
          break;
        }
      }
      if (!all_points_in_front) {
        std::lock_guard<std::mutex> lock(io_mutex);
        std::printf("\n");
        spdlog::warn(
            "PnP result for frame {} has points behind camera, "
            "skipping.",
            base_idx);
        continue;
      }

      // Check 3: Verify reprojection error is reasonable.
      std::vector<cv::Point2f> reprojected_pts;
      cv::projectPoints(obj_pts, r_vec, t_vec, K_eye, D_zero, reprojected_pts);
      double total_reproj_error = 0.0;
      for (size_t i = 0; i < undistorted_pts.size(); ++i) {
        double dx = reprojected_pts[i].x - undistorted_pts[i].x;
        double dy = reprojected_pts[i].y - undistorted_pts[i].y;
        total_reproj_error += std::sqrt(dx * dx + dy * dy);
      }
      double mean_reproj_error = total_reproj_error / undistorted_pts.size();

      // Threshold for normalized coordinates (stricter than pixel error)
      constexpr double kMaxMeanReprojError = 0.1;  // in normalized coordinates
      if (mean_reproj_error > kMaxMeanReprojError) {
        std::lock_guard<std::mutex> lock(io_mutex);
        std::printf("\n");
        spdlog::warn(
            "PnP result for frame {} has high reprojection error: "
            "{:.4f}, skipping.",
            base_idx, mean_reproj_error);
        continue;
      }

      // ===========================================================================

      // Step 2.5: Convert PnP result (transform form world to camera) to our
      // format.
      // r_vec / t_vec represents T_Ci_W.
      Eigen::Vector3d t_Ci_W(t_vec.at<double>(0), t_vec.at<double>(1),
                             t_vec.at<double>(2));
      cv::Mat R_cv;
      cv::Rodrigues(r_vec, R_cv);
      Eigen::Matrix3d R_Ci_W;
      // clang-format off
      R_Ci_W << R_cv.at<double>(0, 0), R_cv.at<double>(0, 1), R_cv.at<double>(0, 2), 
                R_cv.at<double>(1, 0), R_cv.at<double>(1, 1), R_cv.at<double>(1, 2), 
                R_cv.at<double>(2, 0), R_cv.at<double>(2, 1), R_cv.at<double>(2, 2);
      // clang-format on

      Sophus::SE3d T_Ci_W(Eigen::Quaterniond(R_Ci_W).normalized(), t_Ci_W);

      // Transform from the selected camera to the base camera if necessary.
      // We want T_Cb_W.
      Sophus::SE3d T_Cb_W;
      if (best_cam_label == calib_parameters_->base_camera_label) {
        T_Cb_W = T_Ci_W;
      } else {
        // T_Cb_W = T_Cb_Ci * T_Ci_W
        Sophus::SE3d T_Cb_Ci(calib_parameters_->rot_Cb_Ci.at(best_cam_label),
                             calib_parameters_->trans_Cb_Ci.at(best_cam_label));
        T_Cb_W = T_Cb_Ci * T_Ci_W;
      }

      // We store the pose of the base camera in the world frame (T_W_Cb).
      Sophus::SE3d T_W_Cb = T_Cb_W.inverse();
      {
        std::lock_guard<std::mutex> lock(cam_poses_mutex);
        cam_poses_W_Cb[base_idx] = {T_W_Cb.translation(),
                                    T_W_Cb.unit_quaternion()};
        valid_pose_count++;
      }

      // Print progress
      int processed = ++processed_count;
      if (processed % 10 == 0 || processed == static_cast<int>(total_tasks)) {
        std::lock_guard<std::mutex> lock(io_mutex);
        std::printf("\rInitializing camera poses: %d/%zu", processed,
                    total_tasks);
        std::fflush(stdout);
      }
    }
  };

  // ===========================================================================

  // Step 3: Multi-threaded processing.
  std::vector<std::thread> thread_pool;
  int task_step = (total_tasks + kMultiThreadNum - 1) / kMultiThreadNum;
  for (int i = 0; i < kMultiThreadNum; ++i) {
    int start = i * task_step;
    int end = std::min(start + task_step, static_cast<int>(total_tasks));
    if (start < end) {
      thread_pool.emplace_back(worker_func, start, end);
    }
  }

  for (auto& thread : thread_pool) {
    thread.join();
  }
  std::printf("\n");

  constexpr int kMinValidPoses = 10;
  if (valid_pose_count < kMinValidPoses) {
    spdlog::warn(
        "Too few camera poses initialized ({}), calibration might fail or be "
        "unstable.",
        valid_pose_count.load());
    if (valid_pose_count == 0) {
      return false;
    }
  }

  // ===========================================================================

  return true;
}

bool CamRigCalibrator::BuildAndOptimizeCeresProblem(
    const FrameCorrespondence& frame_correspondence, CamPoses& cam_poses_W_Cb) {
  // Step 1: Validate inputs and prepare Ceres problem.
  if (frame_correspondence.empty()) {
    spdlog::error("Frame correspondence is empty, cannot build Ceres problem.");
  }

  if (cam_poses_W_Cb.empty()) {
    spdlog::error("Camera poses are empty, please initialize them first.");
    return false;
  }

  // Prepare Ceres problem, loss function, and quaternion manifolds.
  ceres::Problem problem;
  ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
  ceres::Manifold* quaternion_manifold = new ceres::EigenQuaternionManifold();

  // Prepare target 3D target corners map.
  const auto& target_corners = sensor_manager_->GetTargetCorners()->corners;

  // ===========================================================================

  // Step 2: Apply priors to calibration parameters before optimization.
  const auto& cam_configs = sensor_manager_->GetAllCamConfigs();
  for (const auto& [cam_label, cam_config] : cam_configs) {
    if (cam_config.fix_spatial_extrinsic) {
      calib_parameters_->trans_Cb_Ci.at(cam_label) =
          cam_config.trans_Cb_Ci_prior;
      calib_parameters_->rot_Cb_Ci.at(cam_label) =
          Sophus::SO3d(cam_config.rot_q_Cb_Ci_prior);
    }
    if (cam_config.fix_intrinsic) {
      calib_parameters_->cam_intrinsics.at(cam_label)->parameters =
          cam_config.intrinsic_prior;
    }
  }

  const auto& target_configs = sensor_manager_->GetAllTargetConfigs();
  for (const auto& [target_idx, target_config] : target_configs) {
    if (target_config.fix_spatial_extrinsic) {
      calib_parameters_->trans_W_Ti.at(target_idx) =
          target_config.trans_W_T_prior;
      calib_parameters_->rot_W_Ti.at(target_idx) =
          Sophus::SO3d(target_config.rot_q_W_T_prior);
    }
  }

  // ===========================================================================

  // Step 3: Iterate through all frames and build ceres problem.
  // Step 3.1: Iterate through all initialized base camera poses.
  for (auto& [frame_idx, pose_pair] : cam_poses_W_Cb) {
    // Prepare base camera pose data.
    auto trans_W_Cb_data = pose_pair.first.data();
    auto rot_W_Cb_data = pose_pair.second.coeffs().data();

    // Explicitly add parameter block for base pose orientation with manifold.
    problem.AddParameterBlock(trans_W_Cb_data, 3);
    problem.AddParameterBlock(rot_W_Cb_data, 4, quaternion_manifold);

    // For each base camera frame, find corresponding camera frames.
    auto frame_iter = frame_correspondence.find(frame_idx);
    if (frame_iter == frame_correspondence.end()) {
      spdlog::warn(
          "Frame index {} not found in frame correspondence, skipping.",
          frame_idx);
      continue;
    }
    const auto& cam_frames_corr = frame_iter->second;

    // Step 3.2: Iterate through all corresponding camera frames.
    for (const auto& [cam_label, frame_ptr] : cam_frames_corr) {
      // Prepare camera extrinsic data.
      auto trans_Cb_Ci_data =
          calib_parameters_->trans_Cb_Ci.at(cam_label).data();
      auto rot_Cb_Ci_data = calib_parameters_->rot_Cb_Ci.at(cam_label).data();

      // Prepare camera intrinsic data.
      auto& cam_intrinsic = calib_parameters_->cam_intrinsics.at(cam_label);
      auto intrinsic_data = cam_intrinsic->parameters.data();

      // Step 3.3: Iterate through all keypoints in the frame and add residuals.
      for (const auto& [kp_id, kp] : frame_ptr->keypoints) {
        // Find corresponding 3D target point.
        if (target_corners.count(kp_id) == 0) {
          spdlog::warn("Keypoint ID {} not found in target corners, skipping.",
                       kp_id);
          continue;
        }
        const auto& target_corner = target_corners.at(kp_id);
        int target_idx = target_corner.target_idx;

        // Prepare target pose data.
        auto trans_W_Ti_data =
            calib_parameters_->trans_W_Ti.at(target_idx).data();
        auto rot_W_Ti_data = calib_parameters_->rot_W_Ti.at(target_idx).data();

        // Create cost function and add residual block.
        double weight = 1.0 / cam_configs.at(cam_label).noise;
        ceres::CostFunction* cost_function = CamReprojCost::Create(
            cam_intrinsic, target_corner.position_local, kp, weight);

        problem.AddResidualBlock(cost_function, loss_function, trans_W_Cb_data,
                                 rot_W_Cb_data, trans_Cb_Ci_data,
                                 rot_Cb_Ci_data, intrinsic_data,
                                 trans_W_Ti_data, rot_W_Ti_data);
      }
    }
  }

  // ===========================================================================

  // Step 4: Set parameter blocks constant/manifold for cameras and targets.
  // Step 4.1: Iterate through all cameras and targets to set parameter blocks.
  for (const auto& [cam_label, cam_config] : cam_configs) {
    auto trans_Cb_Ci_data = calib_parameters_->trans_Cb_Ci.at(cam_label).data();
    auto rot_Cb_Ci_data = calib_parameters_->rot_Cb_Ci.at(cam_label).data();
    auto intrinsic_data =
        calib_parameters_->cam_intrinsics.at(cam_label)->parameters.data();

    if (problem.HasParameterBlock(trans_Cb_Ci_data)) {
      if (cam_config.fix_spatial_extrinsic) {
        problem.SetParameterBlockConstant(trans_Cb_Ci_data);
      }
    }

    if (problem.HasParameterBlock(rot_Cb_Ci_data)) {
      problem.SetManifold(rot_Cb_Ci_data, quaternion_manifold);
      if (cam_config.fix_spatial_extrinsic) {
        problem.SetParameterBlockConstant(rot_Cb_Ci_data);
      }
    }

    if (problem.HasParameterBlock(intrinsic_data)) {
      if (cam_config.fix_intrinsic) {
        problem.SetParameterBlockConstant(intrinsic_data);
      }
    }
  }

  for (const auto& [target_idx, target_config] : target_configs) {
    auto trans_W_Ti_data = calib_parameters_->trans_W_Ti.at(target_idx).data();
    auto rot_W_Ti_data = calib_parameters_->rot_W_Ti.at(target_idx).data();

    if (problem.HasParameterBlock(trans_W_Ti_data)) {
      if (target_config.fix_spatial_extrinsic) {
        problem.SetParameterBlockConstant(trans_W_Ti_data);
      }
    }

    if (problem.HasParameterBlock(rot_W_Ti_data)) {
      problem.SetManifold(rot_W_Ti_data, quaternion_manifold);
      if (target_config.fix_spatial_extrinsic) {
        problem.SetParameterBlockConstant(rot_W_Ti_data);
      }
    }
  }

  // Step 4.2: Always fix the base camera and world target poses to
  // eliminate gauge freedom.
  problem.SetParameterBlockConstant(
      calib_parameters_->trans_Cb_Ci.at(calib_parameters_->base_camera_label)
          .data());
  problem.SetParameterBlockConstant(
      calib_parameters_->rot_Cb_Ci.at(calib_parameters_->base_camera_label)
          .data());

  problem.SetParameterBlockConstant(
      calib_parameters_->trans_W_Ti.at(calib_parameters_->world_target_index)
          .data());
  problem.SetParameterBlockConstant(
      calib_parameters_->rot_W_Ti.at(calib_parameters_->world_target_index)
          .data());

  // ===========================================================================

  // Step 5: Configure and run the Ceres solver.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.max_num_iterations =
      system_config_->cam_calib_config.ceres_max_iterations;
  options.num_threads = system_config_->cam_calib_config.multi_thread_num;
  options.minimizer_progress_to_stdout = true;
  FLAGS_minloglevel = google::GLOG_WARNING;  // Suppress Ceres info logs.

  spdlog::info("Running camera rig calibration, it may take a while...");
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // ===========================================================================

  return summary.IsSolutionUsable();
}

void CamRigCalibrator::PrintCalibrationResults() {
  spdlog::info("Base camera: {}, world target: {}",
               calib_parameters_->base_camera_label,
               calib_parameters_->world_target_index);

  // Print camera calibration results.
  for (const auto& [label, _] : sensor_manager_->GetAllCamConfigs()) {
    spdlog::info(" - Results for camera: {}", label);

    Eigen::Vector3d trans_Cb_Ci = calib_parameters_->trans_Cb_Ci.at(label);
    Eigen::Quaterniond rot_Cb_Ci =
        calib_parameters_->rot_Cb_Ci.at(label).unit_quaternion();
    CamIntrinsicBase::Ptr cam_intrinsic =
        calib_parameters_->cam_intrinsics.at(label);

    spdlog::info("Translation Ci in Cb: [{:.6f}, {:.6f}, {:.6f}]",
                 trans_Cb_Ci.x(), trans_Cb_Ci.y(), trans_Cb_Ci.z());
    spdlog::info(
        "Rotation from Ci to Cb (quat_xyzw): [{:.6f}, {:.6f}, {:.6f}, {:.6f}]",
        rot_Cb_Ci.x(), rot_Cb_Ci.y(), rot_Cb_Ci.z(), rot_Cb_Ci.w());
    spdlog::info("Intrinsic parameters: {}",
                 cam_intrinsic->IntrinsicsToString());
  }

  // Print target relative poses.
  for (const auto& [idx, _] : sensor_manager_->GetAllTargetConfigs()) {
    spdlog::info(" - Results for target: {}", idx);

    Eigen::Vector3d trans_W_Ti = calib_parameters_->trans_W_Ti.at(idx);
    Eigen::Quaterniond rot_W_Ti =
        calib_parameters_->rot_W_Ti.at(idx).unit_quaternion();
    spdlog::info("Translation Ti in W: [{:.6f}, {:.6f}, {:.6f}]",
                 trans_W_Ti.x(), trans_W_Ti.y(), trans_W_Ti.z());

    spdlog::info(
        "Rotation from Ti to W (quat_xyzw): [{:.6f}, {:.6f}, {:.6f}, {:.6f}]",
        rot_W_Ti.x(), rot_W_Ti.y(), rot_W_Ti.z(), rot_W_Ti.w());
  }
}

}  // namespace xr_ucalib

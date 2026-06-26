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
#include "xr_ucalib/uc_unified_calib/calibrator/unified_calibrator.h"

#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include <ceres/ceres.h>
#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>

#include "xr_ucalib/uc_unified_calib/calibrator/calibration_validator.h"
#include "xr_ucalib/uc_unified_calib/calibrator/problem_builder.h"
#include "xr_ucalib/uc_unified_calib/initializer/i2p_extrinsic_initializer.h"
#include "xr_ucalib/uc_unified_calib/initializer/m2p_extrinsic_initializer.h"
// clang-format on

namespace xr_ucalib {

bool UnifiedCalibrator::RunCalibration() {
  // We require that SFM calibration has been completed before unified
  // calibration.
  uint8_t required_status = CalibParameters::ParamStatus::SETUP |
                            CalibParameters::ParamStatus::SFM_CALIB;
  if ((calib_parameters_->param_status & required_status) != required_status) {
    spdlog::error(
        "SFM calibration must be completed before unified calibration. "
        "Current param_status: {}",
        static_cast<int>(calib_parameters_->param_status));
    return false;
  }

  // Step 1: Linear initialization for unified calibration.
  spdlog::info("---------------- Linear Initialization ----------------");

  if (!Initialize()) {
    spdlog::error("Failed to initialize the unified calibration states.");
    return false;
  }

  // Step 2: Nonlinear optimization for unified calibration.
  spdlog::info("---------------- Nonlinear Optimization ---------------");

  if (!BuildAndOptimizeCereProblem()) {
    spdlog::error("Failed to build and optimize the Ceres problem.");
    return false;
  }
  spdlog::info("Unified calibration completed.");

  // Step 3: Validate and save the calibration results.
  if (system_config_->workspace_dir == "") {
    spdlog::error(
        "Workspace directory is not set in the system configuration.");
    return false;
  }
  std::string work_dir = system_config_->workspace_dir + "/unified_calib";
  if (!std::filesystem::exists(work_dir)) {
    std::filesystem::create_directories(work_dir);
  }

  std::string traj_output_path = work_dir + "/base_trajectory.txt";
  if (!SaveSystemTrajectory(traj_output_path)) {
    spdlog::error("Failed to save the calibrated system trajectory.");
    return false;
  }

  std::string target_corners_output_path = work_dir + "/target_corners_3d.txt";
  if (!SaveTargetCorners(target_corners_output_path)) {
    spdlog::error("Failed to save the calibrated target corners.");
    return false;
  }

  // Step 4: Print the calibration results.
  spdlog::info("------------- Unified Calibration Results -------------");
  PrintCalibrationResults();

  return true;
}

bool UnifiedCalibrator::Initialize() {
  // Step 1: Initialize the base camera trajectory, used for subsequent
  // pose-centered initialization.
  std::string body_frame_label = calib_parameters_->body_frame_label;
  std::string base_camera_label = calib_parameters_->base_camera_label;
  if (body_frame_label.empty()) {
    spdlog::error(
        "Body frame label is empty in calibration parameters, unable to "
        "perform unified calibration.");
    return false;
  }
  if (base_camera_label.empty()) {
    spdlog::error(
        "Base camera label is empty in calibration parameters, unable to "
        "perform unified calibration.");
    return false;
  }

  // Base camera trajectory (Transformation from base camera to world).
  PoseSequence::Ptr base_cam_traj;
  if (!InitializeBaseTrajectory(base_camera_label, base_cam_traj)) {
    spdlog::error("Failed to initialize base camera trajectory.");
    return false;
  }

  // ===========================================================================

  // Step 2: Initialize the time-invariant calibration parameters in our system.
  // For extrinsic parameters, we use the pose-centered strategy, i.e., first
  // estimate the extrinsics between base camera (Cb) and other sensors, then
  // propagate to body frame (B).
  Eigen::Vector3d trans_Cb_B, gravity_in_W;
  Eigen::Quaterniond rot_q_Cb_B;
  std::map<std::string, Eigen::Vector3d> trans_Cb_Ii_map;
  std::map<std::string, Eigen::Quaterniond> rot_q_Cb_Ii_map;

  auto i2p_initializer = I2PExtrinsicInitializer::Create();
  auto m2p_initializer = M2PExtrinsicInitializer::Create();

  // Step 2.1: Iterate all IMU sensors to estimate extrinsics.
  for (const auto& [label, imu_seq] : sensor_manager_->GetAllImuSequences()) {
    Eigen::Vector3d trans_Cb_Ii;
    Eigen::Quaterniond rot_q_Cb_Ii;
    Eigen::Vector3d gravity;
    bool success = i2p_initializer->EstimateFromSeq(
        base_cam_traj, imu_seq, 0.,
        system_config_->unified_calib_config.gravity_magnitude, trans_Cb_Ii,
        rot_q_Cb_Ii, gravity);

    if (!success) {
      spdlog::error("Failed to initialize extrinsics for IMU: {}", label);
      return false;
    }

    if (label == body_frame_label) {
      trans_Cb_B = trans_Cb_Ii;
      rot_q_Cb_B = rot_q_Cb_Ii;
      gravity_in_W = gravity;
    } else {
      trans_Cb_Ii_map[label] = trans_Cb_Ii;
      rot_q_Cb_Ii_map[label] = rot_q_Cb_Ii;
    }
  }

  // Propagate the extrinsics to body frame and save in calibration parameters.
  calib_parameters_->trans_B_Cb = -(rot_q_Cb_B.inverse() * trans_Cb_B);
  calib_parameters_->rot_B_Cb = Sophus::SO3d(rot_q_Cb_B.inverse());
  calib_parameters_->gravity_in_W = gravity_in_W;
  for (const auto& [label, _] : sensor_manager_->GetAllImuSequences()) {
    if (label == body_frame_label) continue;
    calib_parameters_->trans_B_Ii[label] =
        calib_parameters_->rot_B_Cb * trans_Cb_Ii_map.at(label) +
        calib_parameters_->trans_B_Cb;
    calib_parameters_->rot_B_Ii.at(label) =
        calib_parameters_->rot_B_Cb * Sophus::SO3d(rot_q_Cb_Ii_map.at(label));
  }

  // Step 2.2: Iterate all magnetometer sensors to estimate extrinsics.
  for (const auto& [label, mag_seq] : sensor_manager_->GetAllMagSequences()) {
    Eigen::Quaterniond rot_q_Cb_Mi;
    Eigen::Vector3d mag_field;
    bool success = m2p_initializer->EstimateFromSeq(base_cam_traj, mag_seq, 0.,
                                                    rot_q_Cb_Mi, mag_field);

    if (!success) {
      spdlog::error("Failed to initialize extrinsics for Magnetometer: {}",
                    label);
      return false;
    }

    calib_parameters_->rot_B_Mi[label] =
        calib_parameters_->rot_B_Cb * Sophus::SO3d(rot_q_Cb_Mi);
    calib_parameters_->mag_in_W = mag_field;
  }

  // ===========================================================================

  // Step 3: Determine the start / end time for the system splines.
  double spline_start_time = sensor_manager_->GetCommonStartTime();
  double spline_end_time = sensor_manager_->GetCommonEndTime();
  if (spline_start_time >= spline_end_time) {
    spdlog::error(
        "Invalid common time interval for all sensors: [{}, {}], please "
        "check if the sensors share a common clock.",
        spline_start_time, spline_end_time);
    return false;
  }

  // Clamp spline range to the recovered base trajectory time range.
  if (!base_cam_traj || base_cam_traj->Size() < 2) {
    spdlog::error("Base camera trajectory is empty, cannot initialize splines.");
    return false;
  }
  const double traj_start_time = base_cam_traj->Front()->timestamp;
  const double traj_end_time = base_cam_traj->Back()->timestamp;
  spline_start_time = std::max(spline_start_time, traj_start_time);
  spline_end_time = std::min(spline_end_time, traj_end_time);

  // Consider the time margin.
  const double kKnotInterval =
      system_config_->unified_calib_config.spline_knot_interval;
  const double kSplineKnotDeltaTime =
      static_cast<double>(kSplineOrder - 2) / 2. * kKnotInterval;
  double time_margin = kSplineKnotDeltaTime + kKnotInterval +
                       system_config_->unified_calib_config.max_toff_change +
                       0.01;  // Extra 10 ms margin.
  spline_start_time += time_margin;
  spline_end_time -= time_margin;
  if (spline_start_time < 0 || spline_end_time < 0 ||
      spline_start_time >= spline_end_time) {
    spdlog::error(
        "Invalid start / end time [{:.6f}, {:.6f}] for the system splines, "
        "please check "
        "the sensor data timestamps.",
        spline_start_time, spline_end_time);
    return false;
  }

  // ===========================================================================

  // Step 3: Initialize the time-varying parameters: continuous-time splines.
  std::string trans_spline_name = "trans_spline";
  std::string rot_spline_name = "rot_spline";
  trans_spline_info_ =
      SplineInfo(trans_spline_name, SplineType::EuclideanSpline,
                 spline_start_time, spline_end_time, kKnotInterval);
  rot_spline_info_ =
      SplineInfo(rot_spline_name, SplineType::So3Spline, spline_start_time,
                 spline_end_time, kKnotInterval);
  spline_bundle_ =
      SplineBundleType::Create({trans_spline_info_, rot_spline_info_});

  auto& trans_spline = spline_bundle_->GetR3dSpline(trans_spline_name);
  auto& rot_spline = spline_bundle_->GetSo3dSpline(rot_spline_name);
  spline_start_time = trans_spline.MinTime();
  spline_end_time = trans_spline.MaxTime();
  size_t knot_size = trans_spline.get_knots().size();

  // Check the time
  if (!(std::abs(trans_spline.MinTime() - rot_spline.MinTime()) < 1.e-9) ||
      !(std::abs(trans_spline.MaxTime() - rot_spline.MaxTime()) < 1.e-9) ||
      trans_spline.get_knots().size() != rot_spline.get_knots().size()) {
    spdlog::error("Inconsistent parameters for system B-splines. ");
    return false;
  } else if ((spline_end_time - spline_start_time) <
             2 * kSplineOrder * time_margin) {
    spdlog::error(
        "The time range for optimization is too small. Please input "
        "measurement data covering a longer time period. ");
    return false;
  }
  trans_spline_info_.start_time = spline_start_time;
  trans_spline_info_.end_time = spline_end_time;
  rot_spline_info_.start_time = spline_start_time;
  rot_spline_info_.end_time = spline_end_time;

  // Initialize the spline control points using the base camera trajectory.
  // Note that the spline represents the transformation from body frame (B)
  // to world frame (W).
  double current_knot_time = spline_start_time - kSplineKnotDeltaTime;
  for (size_t k = 0; k < knot_size; ++k) {
    Eigen::Vector3d trans_W_Cb;
    Eigen::Quaterniond rot_q_W_Cb;

    if (!GetTargetPose(base_cam_traj, current_knot_time, trans_W_Cb,
                       rot_q_W_Cb)) {
      spdlog::error(
          "Failed to get base camera pose at time {:.6f} for spline "
          "initialization. ",
          current_knot_time);
      return false;
    }

    Eigen::Vector3d knot_trans;
    Eigen::Quaterniond knot_rot_q;
    // T_G_B = T_W_Cb * T_Cb_B.
    knot_trans = rot_q_W_Cb * trans_Cb_B + trans_W_Cb;
    knot_rot_q = rot_q_W_Cb * rot_q_Cb_B;

    trans_spline.get_knot(k) = knot_trans;
    rot_spline.get_knot(k) = Sophus::SO3d(knot_rot_q);

    current_knot_time += kKnotInterval;
  }

  spdlog::info(
      "Initialize system splines with {} knots over time range [{:.6f}, "
      "{:.6f}].",
      knot_size, spline_start_time, spline_end_time);

  // ===========================================================================

  // Step 4: Print the initialized calibration parameters.
  spdlog::info("Linear initialization results:");
  spdlog::info("Body frame: {}, base camera: {}, world target: {}",
               body_frame_label, base_camera_label,
               calib_parameters_->world_target_index);

  // Print base camera extrinsics.
  spdlog::info(" - Linear initialization for base camera: {}",
               base_camera_label);

  Eigen::Vector3d trans_B_Cb_print = calib_parameters_->trans_B_Cb;
  Eigen::Quaterniond rot_B_Cb_print =
      calib_parameters_->rot_B_Cb.unit_quaternion();
  spdlog::info("Translation Cb in B: [{:.6f}, {:.6f}, {:.6f}]",
               trans_B_Cb_print.x(), trans_B_Cb_print.y(),
               trans_B_Cb_print.z());
  spdlog::info(
      "Rotation from Cb to B (quat_xyzw): [{:.6f}, {:.6f}, {:.6f}, {:.6f}]",
      rot_B_Cb_print.x(), rot_B_Cb_print.y(), rot_B_Cb_print.z(),
      rot_B_Cb_print.w());

  // Print system gravity direction.
  Eigen::Vector3d gravity_in_W_print = calib_parameters_->gravity_in_W;
  spdlog::info("Gravity direction in world frame W: [{:.6f}, {:.6f}, {:.6f}]",
               gravity_in_W_print.x(), gravity_in_W_print.y(),
               gravity_in_W_print.z());

  // Print IMU extrinsics.
  for (const auto& [label, _] : sensor_manager_->GetAllImuSequences()) {
    spdlog::info(" - Linear initialization for IMU: {}", label);

    Eigen::Vector3d trans_B_Ii_print = calib_parameters_->trans_B_Ii.at(label);
    Eigen::Quaterniond rot_B_Ii_print =
        calib_parameters_->rot_B_Ii.at(label).unit_quaternion();
    spdlog::info("Translation Ii in B: [{:.6f}, {:.6f}, {:.6f}]",
                 trans_B_Ii_print.x(), trans_B_Ii_print.y(),
                 trans_B_Ii_print.z());
    spdlog::info(
        "Rotation from Ii to B (quat_xyzw): [{:.6f}, {:.6f}, {:.6f}, {:.6f}]",
        rot_B_Ii_print.x(), rot_B_Ii_print.y(), rot_B_Ii_print.z(),
        rot_B_Ii_print.w());
  }

  // Print magnetometer extrinsics.
  for (const auto& [label, _] : sensor_manager_->GetAllMagSequences()) {
    spdlog::info(" - Linear initialization for Magnetometer: {}", label);

    Sophus::SO3d rot_B_Mi_print = calib_parameters_->rot_B_Mi.at(label);
    spdlog::info(
        "Rotation from Mi to B (quat_xyzw): [{:.6f}, {:.6f}, {:.6f}, {:.6f}]",
        rot_B_Mi_print.unit_quaternion().x(),
        rot_B_Mi_print.unit_quaternion().y(),
        rot_B_Mi_print.unit_quaternion().z(),
        rot_B_Mi_print.unit_quaternion().w());
  }

  if (!sensor_manager_->GetAllMagSequences().empty()) {
    Eigen::Vector3d mag_in_W_print = calib_parameters_->mag_in_W;
    spdlog::info(
        "Magnetic field direction in world frame W: [{:.6f}, {:.6f}, {:.6f}]",
        mag_in_W_print.x(), mag_in_W_print.y(), mag_in_W_print.z());
  }

  // ===========================================================================

  return true;
}

bool UnifiedCalibrator::BuildAndOptimizeCereProblem() {
  // Extend time range of sensor measurements to cover entire B-spline after
  // time offsets are optimized. For measurements outside the B-spline time
  // range, we simply set their gradients to zero.
  const double max_toff_change =
      system_config_->unified_calib_config.max_toff_change;
  const double kMeasStartTime = trans_spline_info_.start_time - max_toff_change;
  const double kMeasEndTime = trans_spline_info_.end_time + max_toff_change;

  // Step 1: Initialize the problem builder and Ceres problem.
  ProblemBuilder::Context problem_builder_context;
  problem_builder_context.system_config = system_config_;
  problem_builder_context.calib_parameters = calib_parameters_;
  problem_builder_context.spline_bundle = spline_bundle_;
  problem_builder_context.trans_spline_info = trans_spline_info_;
  problem_builder_context.rot_spline_info = rot_spline_info_;
  auto problem_builder = ProblemBuilder::Create(problem_builder_context);

  CalibrationValidator::Context validator_context;
  validator_context.system_config = system_config_;
  validator_context.sensor_manager = sensor_manager_;
  validator_context.calib_parameters = calib_parameters_;
  validator_context.spline_bundle = spline_bundle_;
  validator_context.trans_spline_info = trans_spline_info_;
  validator_context.rot_spline_info = rot_spline_info_;
  auto validator = CalibrationValidator::Create(validator_context);

  ceres::Problem::Options problem_options;
  // It is essential to set ownership options to avoid memory issues.
  problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_options);

  // Validation data container:
  // key format is "cam:<label>", "imu:<label>", "mag:<label>".
  std::map<std::string, std::vector<double>> sequence_timestamps;

  // ===========================================================================

  // Step 2: Add residual blocks for all camera frames.
  int cam_residual_count = 0;
  constexpr int kMinKeypoints = 10;
  int ignored_cam_frames = 0;
  const auto& cam_sequences = sensor_manager_->GetAllCamSequences();
  const auto& cam_configs = sensor_manager_->GetAllCamConfigs();
  const auto& target_corners = sensor_manager_->GetTargetCorners();
  const auto& target_configs = sensor_manager_->GetAllTargetConfigs();
  for (const auto& [cam_label, cam_seq] : cam_sequences) {
    sequence_timestamps["cam:" + cam_label] = {};
    cam_residual_count = 0;
    int frame_idx = 0;
    const int down_sample_rate =
        cam_configs.at(cam_label).down_sample_rate_ucalib;

    for (const auto& cam_frame : cam_seq->frames) {
      if ((frame_idx++) % down_sample_rate != 0) continue;

      if (cam_frame->keypoints.size() < kMinKeypoints) {
        ignored_cam_frames++;
        continue;
      }

      if (cam_frame->timestamp < kMeasStartTime ||
          cam_frame->timestamp > kMeasEndTime) {
        continue;
      }

      if (!problem_builder->AddCamReprojResiduals(
              problem, cam_label, cam_frame, target_corners,
              cam_configs.at(cam_label), target_configs)) {
        spdlog::error("Failed to add residuals for camera frame at time {:.6f}",
                      cam_frame->timestamp);
        return false;
      }

      ++cam_residual_count;
      sequence_timestamps["cam:" + cam_label].push_back(cam_frame->timestamp);
    }

    if (ignored_cam_frames > cam_residual_count * 0.1) {
      spdlog::warn(
          "Camera {} has {} frames with less than {} keypoints, which may "
          "affect the calibration accuracy.",
          cam_label, ignored_cam_frames, kMinKeypoints);
    }

    spdlog::info("Added {}  residual blocks for camera: {}", cam_residual_count,
                 cam_label);
  }

  // ===========================================================================

  // Step 3: Add residual blocks for all IMU frames.
  int imu_residual_count = 0;
  const auto& imu_sequences = sensor_manager_->GetAllImuSequences();
  const auto& imu_configs = sensor_manager_->GetAllImuConfigs();
  for (const auto& [imu_label, imu_seq] : imu_sequences) {
    sequence_timestamps["imu:" + imu_label] = {};
    imu_residual_count = 0;
    int frame_idx = 0;
    const int down_sample_rate =
        imu_configs.at(imu_label).down_sample_rate_ucalib;

    for (const auto& imu_frame : imu_seq->frames) {
      if ((frame_idx++) % down_sample_rate != 0) continue;

      if (imu_frame->timestamp < kMeasStartTime ||
          imu_frame->timestamp > kMeasEndTime) {
        continue;
      }

      if (!problem_builder->AddImuAccResiduals(problem, imu_label, imu_frame,
                                               imu_configs.at(imu_label))) {
        spdlog::error("Failed to add accelerometer residuals at time {:.6f}",
                      imu_frame->timestamp);
        return false;
      }

      if (!problem_builder->AddImuGyrResiduals(problem, imu_label, imu_frame,
                                               imu_configs.at(imu_label))) {
        spdlog::error("Failed to add gyroscope residuals at time {:.6f}",
                      imu_frame->timestamp);
        return false;
      }

      ++imu_residual_count;
      sequence_timestamps["imu:" + imu_label].push_back(imu_frame->timestamp);
    }

    spdlog::info("Added {} residual blocks for IMU: {}", imu_residual_count,
                 imu_label);
  }

  // ===========================================================================

  // Step 4: Add residual blocks for all magnetometer frames.
  int mag_residual_count = 0;
  const auto& mag_sequences = sensor_manager_->GetAllMagSequences();
  const auto& mag_configs = sensor_manager_->GetAllMagConfigs();
  for (const auto& [mag_label, mag_seq] : mag_sequences) {
    sequence_timestamps["mag:" + mag_label] = {};
    mag_residual_count = 0;
    int frame_idx = 0;
    const int down_sample_rate =
        mag_configs.at(mag_label).down_sample_rate_ucalib;

    for (const auto& mag_frame : mag_seq->frames) {
      if ((frame_idx++) % down_sample_rate != 0) continue;

      if (mag_frame->timestamp < kMeasStartTime ||
          mag_frame->timestamp > kMeasEndTime) {
        continue;
      }

      if (!problem_builder->AddMagResiduals(problem, mag_label, mag_frame,
                                            mag_configs.at(mag_label))) {
        spdlog::error("Failed to add magnetometer residuals at time {:.6f}",
                      mag_frame->timestamp);
        return false;
      }

      ++mag_residual_count;
      sequence_timestamps["mag:" + mag_label].push_back(mag_frame->timestamp);
    }

    spdlog::info("Added {} residual blocks for Magnetometer: {}",
                 mag_residual_count, mag_label);
  }

  // ===========================================================================

  // Step 5: Configure and run the Ceres solver.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.max_num_iterations =
      system_config_->unified_calib_config.ceres_max_iterations;
  options.num_threads = system_config_->unified_calib_config.multi_thread_num;
  options.minimizer_progress_to_stdout = true;
  FLAGS_minloglevel = google::GLOG_WARNING;  // Suppress Ceres info logs.

  spdlog::info("Runing unified calibration, it may take a while...");
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // ===========================================================================

  // Step 6: Validate and save quality report.
  if (!system_config_->workspace_dir.empty()) {
    std::string work_dir = system_config_->workspace_dir + "/unified_calib";
    if (!std::filesystem::exists(work_dir)) {
      std::filesystem::create_directories(work_dir);
    }

    std::string validation_output_path = work_dir + "/validation_report.txt";
    if (!validator->ValidateAndSaveResults(validation_output_path,
                         sequence_timestamps,
                         kMeasStartTime, kMeasEndTime,
                         summary.IsSolutionUsable())) {
      spdlog::warn("Failed to generate validation report: {}",
                   validation_output_path);
    } else {
      spdlog::info("Validation report saved to: {}", validation_output_path);
    }
  } else {
    spdlog::warn(
        "Workspace directory is empty, skip writing unified calibration "
        "validation report.");
  }

  // ===========================================================================

  return summary.IsSolutionUsable();
}

bool UnifiedCalibrator::InitializeBaseTrajectory(
    std::string base_camera_label, PoseSequence::Ptr& base_cam_traj) {
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

  // Step 2: Collect all camera frames sorted by timestamp.
  struct FrameInfo {
    double timestamp;
    std::string cam_label;
    CamFrame::Ptr frame;
  };
  std::vector<FrameInfo> all_frames;
  const auto& all_cam_sequences = sensor_manager_->GetAllCamSequences();
  for (const auto& [cam_label, cam_seq] : all_cam_sequences) {
    if (!cam_seq) continue;
    for (const auto& frame : cam_seq->frames) {
      if (frame && !frame->keypoints.empty()) {
        all_frames.push_back({frame->timestamp, cam_label, frame});
      }
    }
  }

  if (all_frames.empty()) {
    spdlog::error("No camera frames found for trajectory initialization.");
    return false;
  }

  std::sort(all_frames.begin(), all_frames.end(),
            [](const FrameInfo& a, const FrameInfo& b) {
              return a.timestamp < b.timestamp;
            });

  // ===========================================================================

  // Step 3: Select frames with the most keypoints, respecting the interval.
  std::vector<FrameInfo> selected_frames;
  const double kMinInterval =
      system_config_->unified_calib_config.spline_knot_interval;
  const size_t kMinKeypoints = 20;
  int success_count = 0;
  int failure_count = 0;
  const double kSelectionFailureThresholdRatio = 0.15;

  double last_selected_time = -1e9;
  auto it = all_frames.begin();

  while (it != all_frames.end()) {
    // Skip frames that are too close to the last selected frame.
    if (it->timestamp < last_selected_time + kMinInterval) {
      ++it;
      continue;
    }

    // Search for the best frame in the upcoming window.
    double window_start_time = it->timestamp;
    double window_end_time = window_start_time + kMinInterval;

    auto best_it = it;
    size_t max_keypoints = 0;

    auto search_it = it;
    while (search_it != all_frames.end() &&
           search_it->timestamp < window_end_time) {
      if (search_it->frame->keypoints.size() > max_keypoints) {
        max_keypoints = search_it->frame->keypoints.size();
        best_it = search_it;
      }
      ++search_it;
    }

    if (max_keypoints >= kMinKeypoints) {
      selected_frames.push_back(*best_it);
      last_selected_time = best_it->timestamp;
      it = search_it;
      success_count++;
    } else {
      it = search_it;
      failure_count++;
    }
  }

  spdlog::info(
      "Select {} frames from {} total frames for base trajectory "
      "initialization.",
      selected_frames.size(), all_frames.size());

  if (failure_count > 2 * static_cast<int>(kSelectionFailureThresholdRatio *
                                           (success_count + failure_count))) {
    spdlog::error(
        "Excessive failures ({}) in selecting frames for base camera "
        "trajectory initialization, please check keypoint detections.",
        failure_count);
    return false;

  } else if (failure_count >
             static_cast<int>(kSelectionFailureThresholdRatio *
                              (success_count + failure_count))) {
    spdlog::warn(
        "Excessive failures ({}) in selecting frames for base camera "
        "trajectory initialization, please check keypoint detections.",
        failure_count);
  }

  // ===========================================================================

  // Step 4: Compute camera poses using PnP, and build the base camera
  // trajectory. (Prepare parallel tasks.)
  base_cam_traj = PoseSequence::Create();
  std::vector<PoseFrame::Ptr> valid_poses(selected_frames.size(), nullptr);
  std::atomic<int> valid_count(0);
  std::atomic<int> processed_count(0);
  std::mutex io_mutex;

  const int num_threads = system_config_->unified_calib_config.multi_thread_num;
  const int total_tasks = selected_frames.size();
  std::vector<std::thread> thread_pool;

  auto worker_func = [&](int start_idx, int end_idx) {
    for (int i = start_idx; i < end_idx; ++i) {
      const auto& info = selected_frames[i];
      const auto& frame = info.frame;

      // Step 4.1: Prepare 2D-3D correspondences.
      std::vector<cv::Point3d> object_points;
      std::vector<cv::Point2d> image_points;

      if (calib_parameters_->cam_intrinsics.find(info.cam_label) ==
          calib_parameters_->cam_intrinsics.end()) {
        spdlog::error(
            "Camera intrinsic parameters not found for camera {}, skipping "
            "frame at time {:.6f}.",
            info.cam_label, info.timestamp);
        continue;
      }
      auto cam_intr = calib_parameters_->cam_intrinsics.at(info.cam_label);

      for (const auto& [id, kp] : frame->keypoints) {
        if (target_corners_3d.find(id) != target_corners_3d.end()) {
          Eigen::Vector3d p_global = target_corners_3d.at(id).position_global;
          object_points.emplace_back(p_global.x(), p_global.y(), p_global.z());
          image_points.emplace_back(kp.x(), kp.y());
        }
      }

      if (object_points.size() < 6) {
        std::lock_guard<std::mutex> lock(io_mutex);
        printf("\n");
        spdlog::warn(
            "Not enough 2D-3D correspondences ({} points) for PnP in frame "
            "at time {:.6f} of camera {}.",
            object_points.size(), info.timestamp, info.cam_label);
        continue;
      }

      // ===========================================================================

      // Step 4.2: Undistort image points.
      cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
      K.at<double>(0, 0) = cam_intr->parameters[0];  // fx
      K.at<double>(1, 1) = cam_intr->parameters[1];  // fy
      K.at<double>(0, 2) = cam_intr->parameters[2];  // cx
      K.at<double>(1, 2) = cam_intr->parameters[3];  // cy

      cv::Mat D;
      std::vector<cv::Point2d> undistorted_pts;

      if (cam_intr->cam_model_type == CamModelType::RADTAN) {
        // k1, k2, p1, p2
        D = (cv::Mat_<double>(4, 1) << cam_intr->parameters[4],
             cam_intr->parameters[5], cam_intr->parameters[6],
             cam_intr->parameters[7]);
        cv::undistortPoints(image_points, undistorted_pts, K, D);
      } else if (cam_intr->cam_model_type == CamModelType::EQUIDISTANT) {
        // k1, k2, k3, k4
        D = (cv::Mat_<double>(4, 1) << cam_intr->parameters[4],
             cam_intr->parameters[5], cam_intr->parameters[6],
             cam_intr->parameters[7]);
        cv::fisheye::undistortPoints(image_points, undistorted_pts, K, D);
      } else {
        std::lock_guard<std::mutex> lock(io_mutex);
        std::printf("\n");
        spdlog::warn("Unsupported camera model type for PnP initialization: {}",
                     static_cast<int>(cam_intr->cam_model_type));
        continue;
      }

      // Sanity check on undistorted points: for fisheye lenses, undistortPoints
      // outputs pinhole-normalized coords (X/Z, Y/Z). At wide angles (>~75°),
      // these values blow up, causing PnP to fail. Filter out such unreliable
      // points.
      {
        constexpr double kMaxNormalizedCoord = 3.0;  // ~71° from optical axis
        std::vector<cv::Point3d> filtered_obj;
        std::vector<cv::Point2d> filtered_img;
        filtered_obj.reserve(object_points.size());
        filtered_img.reserve(undistorted_pts.size());
        for (size_t j = 0; j < undistorted_pts.size(); ++j) {
          if (std::abs(undistorted_pts[j].x) < kMaxNormalizedCoord &&
              std::abs(undistorted_pts[j].y) < kMaxNormalizedCoord &&
              std::isfinite(undistorted_pts[j].x) &&
              std::isfinite(undistorted_pts[j].y)) {
            filtered_obj.push_back(object_points[j]);
            filtered_img.push_back(undistorted_pts[j]);
          }
        }
        object_points = std::move(filtered_obj);
        undistorted_pts = std::move(filtered_img);
      }

      // ===========================================================================

      // Step 4.3: Solve PnP with identity K and zero D on normalized points.
      cv::Mat K_eye = cv::Mat::eye(3, 3, CV_64F);
      cv::Mat D_zero = cv::Mat::zeros(4, 1, CV_64F);
      cv::Mat r_vec, t_vec;

      // Minimum number of points for reliable PnP.
      constexpr size_t kMinPnPPoints = 6;
      if (object_points.size() < kMinPnPPoints) {
        int processed = ++processed_count;
        if (processed % 10 == 0 || processed == total_tasks) {
          std::lock_guard<std::mutex> lock(io_mutex);
          std::printf("\rInitializing base trajectory: %d/%d", processed,
                      total_tasks);
          std::fflush(stdout);
        }
        continue;
      }

      // Use RANSAC-based PnP for robustness against outliers and noise.
      cv::Mat inlier_mask;
      constexpr double kRansacReprojThreshold = 0.1;
      constexpr int kRansacIterations = 100;
      constexpr double kRansacConfidence = 0.99;

      bool pnp_success = cv::solvePnPRansac(
          object_points, undistorted_pts, K_eye, D_zero, r_vec, t_vec, false,
          kRansacIterations, kRansacReprojThreshold, kRansacConfidence,
          inlier_mask, cv::SOLVEPNP_EPNP);

      if (pnp_success) {
        // Collect inlier points for refinement.
        std::vector<cv::Point3d> inlier_obj_pts;
        std::vector<cv::Point2d> inlier_img_pts;
        for (int j = 0; j < inlier_mask.rows; ++j) {
          if (inlier_mask.at<int>(j)) {
            inlier_obj_pts.push_back(object_points[j]);
            inlier_img_pts.push_back(undistorted_pts[j]);
          }
        }

        // Check inlier ratio - if too low, result is unreliable.
        double inlier_ratio =
            static_cast<double>(inlier_obj_pts.size()) / object_points.size();
        if (inlier_ratio < 0.2 || inlier_obj_pts.size() < kMinPnPPoints) {
          int processed = ++processed_count;
          if (processed % 10 == 0 || processed == total_tasks) {
            std::lock_guard<std::mutex> lock(io_mutex);
            std::printf("\rInitializing base trajectory: %d/%d", processed,
                        total_tasks);
            std::fflush(stdout);
          }
          continue;
        }

        // Refine with inliers only using iterative method.
        cv::solvePnP(inlier_obj_pts, inlier_img_pts, K_eye, D_zero, r_vec,
                     t_vec, true, cv::SOLVEPNP_ITERATIVE);

        // Update to inliers for later validation.
        object_points = std::move(inlier_obj_pts);
        undistorted_pts = std::move(inlier_img_pts);
      } else {
        int processed = ++processed_count;
        if (processed % 10 == 0 || processed == total_tasks) {
          std::lock_guard<std::mutex> lock(io_mutex);
          std::printf("\rInitializing base trajectory: %d/%d", processed,
                      total_tasks);
          std::fflush(stdout);
        }
        continue;
      }

      // ===========================================================================

      // Step 4.4: Validate PnP result before using it.
      // Check 1: Verify that the translation is reasonable (not too far).
      constexpr double kMaxTranslationNorm = 100.0;  // meters
      double trans_norm = cv::norm(t_vec);
      if (trans_norm > kMaxTranslationNorm) {
        int processed = ++processed_count;
        if (processed % 10 == 0 || processed == total_tasks) {
          std::lock_guard<std::mutex> lock(io_mutex);
          std::printf("\rInitializing base trajectory: %d/%d", processed,
                      total_tasks);
          std::fflush(stdout);
        }
        continue;
      }

      // Check 2: Verify that all 3D points have positive depth in camera frame.
      cv::Mat R_cv_check;
      cv::Rodrigues(r_vec, R_cv_check);
      bool all_points_in_front = true;
      for (const auto& pt : object_points) {
        double z_cam = R_cv_check.at<double>(2, 0) * pt.x +
                       R_cv_check.at<double>(2, 1) * pt.y +
                       R_cv_check.at<double>(2, 2) * pt.z + t_vec.at<double>(2);
        if (z_cam <= 0.01) {
          all_points_in_front = false;
          break;
        }
      }
      if (!all_points_in_front) {
        int processed = ++processed_count;
        if (processed % 10 == 0 || processed == total_tasks) {
          std::lock_guard<std::mutex> lock(io_mutex);
          std::printf("\rInitializing base trajectory: %d/%d", processed,
                      total_tasks);
          std::fflush(stdout);
        }
        continue;
      }

      // Check 3: Verify reprojection error is reasonable.
      std::vector<cv::Point2d> reprojected_pts;
      cv::projectPoints(object_points, r_vec, t_vec, K_eye, D_zero,
                        reprojected_pts);
      double total_reproj_error = 0.0;
      for (size_t j = 0; j < undistorted_pts.size(); ++j) {
        double dx = reprojected_pts[j].x - undistorted_pts[j].x;
        double dy = reprojected_pts[j].y - undistorted_pts[j].y;
        total_reproj_error += std::sqrt(dx * dx + dy * dy);
      }
      double mean_reproj_error = total_reproj_error / undistorted_pts.size();

      constexpr double kMaxMeanReprojError = 0.1;  // in normalized coords
      if (mean_reproj_error > kMaxMeanReprojError) {
        int processed = ++processed_count;
        if (processed % 10 == 0 || processed == total_tasks) {
          std::lock_guard<std::mutex> lock(io_mutex);
          std::printf("\rInitializing base trajectory: %d/%d", processed,
                      total_tasks);
          std::fflush(stdout);
        }
        continue;
      }

      // ===========================================================================

      // Step 4.5: Convert PnP result to base camera frame and save.
      {
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

        Sophus::SE3d T_Cb_W;
        if (info.cam_label == calib_parameters_->base_camera_label) {
          T_Cb_W = T_Ci_W;
        } else {
          // T_Cb_W = T_Cb_Ci * T_Ci_W
          Sophus::SE3d T_Cb_Ci(
              calib_parameters_->rot_Cb_Ci.at(info.cam_label),
              calib_parameters_->trans_Cb_Ci.at(info.cam_label));
          T_Cb_W = T_Cb_Ci * T_Ci_W;
        }

        Sophus::SE3d T_W_Cb = T_Cb_W.inverse();

        auto pose_frame = PoseFrame::Create();
        pose_frame->timestamp = info.timestamp;
        pose_frame->trans = T_W_Cb.translation();
        pose_frame->rot_q = T_W_Cb.unit_quaternion();
        valid_poses[i] = pose_frame;

        valid_count++;
      }

      int processed = ++processed_count;
      if (processed % 10 == 0 || processed == total_tasks) {
        std::lock_guard<std::mutex> lock(io_mutex);
        std::printf("\rInitializing base trajectory: %d/%d", processed,
                    total_tasks);
        std::fflush(stdout);
      }
    }
  };

  // ===========================================================================

  // Step 5: Launch threads to process the tasks.
  int task_step = (total_tasks + num_threads - 1) / num_threads;
  for (int i = 0; i < num_threads; ++i) {
    int start = i * task_step;
    int end = std::min(start + task_step, total_tasks);
    if (start >= end) break;
    thread_pool.emplace_back(worker_func, start, end);
  }

  for (auto& t : thread_pool) {
    if (t.joinable()) t.join();
  }
  std::printf("\n");

  constexpr double kPnPFailureThresholdRatio = 0.2;
  const int pnp_success_count = valid_count.load();
  const int failed_count = total_tasks - pnp_success_count;
  const double failure_ratio = static_cast<double>(failed_count) / total_tasks;

  if (failure_ratio > 2 * kPnPFailureThresholdRatio) {
    spdlog::error(
        "PnP failure rate is too high ({:.1f}%, {}/{}). Aborting "
        "initialization.",
        failure_ratio * 100.0, failed_count, total_tasks);

    return false;
  } else if (failure_ratio > kPnPFailureThresholdRatio) {
    spdlog::warn(
        "PnP failure rate is high ({:.1f}%, {}/{}). Initialization might be "
        "unstable.",
        failure_ratio * 100.0, failed_count, total_tasks);
  }

  for (const auto& pose : valid_poses) {
    if (pose) {
      base_cam_traj->Add(pose);
    }
  }

  // ===========================================================================

  spdlog::info("Construct base trajectory with {} poses.",
               base_cam_traj->Size());

  return true;
}

bool UnifiedCalibrator::GetTargetPose(const PoseSequence::Ptr pose_seq,
                                      const double& timestamp,
                                      Eigen::Vector3d& trans,
                                      Eigen::Quaterniond& rot_q) {
  trans = Eigen::Vector3d::Zero();
  rot_q = Eigen::Quaterniond::Identity();

  if (pose_seq->Size() < 2) {
    spdlog::error("Insufficient pose frames to get target pose. ");
    return false;
  }

  // Step 1: Find the bounding poses.
  auto target_ub =
      std::upper_bound(pose_seq->begin(), pose_seq->end(), timestamp,
                       [](double timestamp, const PoseFrame::Ptr& frame) {
                         return timestamp < frame->timestamp;
                       });

  if (target_ub == pose_seq->begin() || target_ub == pose_seq->end()) {
    spdlog::error(
        "Specified pose interpolation time: {:.9f} is out of the range "
        "of the pose data: ({:.9f}, {:.9f}). ",
        timestamp, pose_seq->Front()->timestamp, pose_seq->Back()->timestamp);
    return false;
  }

  PoseFrame::Ptr pose_1 = *target_ub;
  PoseFrame::Ptr pose_0 = *(--target_ub);

  if (std::abs(timestamp - pose_0->timestamp) > 1.0 ||
      std::abs(timestamp - pose_1->timestamp) > 1.0) {
    spdlog::error(
        "Missing pose data for more than 1.0s around {}, unable to "
        "perform an accurate interpolation. ",
        timestamp);
  }

  // Interpolation.
  double lambda =
      (timestamp - pose_0->timestamp) / (pose_1->timestamp - pose_0->timestamp);

  // The translation part is interpolated using LERP.
  trans = (1 - lambda) * pose_0->trans + lambda * pose_1->trans;
  // The rotation component is interpolated using SLERP.
  rot_q = pose_0->rot_q.slerp(lambda, pose_1->rot_q).normalized();

  return true;
}

bool UnifiedCalibrator::SaveSystemTrajectory(const std::string& output_path) {
  if (!spline_bundle_) {
    spdlog::error("System trajectory spline bundle is not initialized.");
    return false;
  }

  std::ofstream traj_file(output_path);
  if (!traj_file.is_open()) {
    spdlog::error("Failed to open file for saving trajectory: {}", output_path);
    return false;
  }
  traj_file << std::fixed << std::setprecision(9);

  constexpr double kOutputInterval = 0.02;  // Default 50Hz output.
  const double kOutputMargin =
      UnifiedCalibConfig::kSplineOrder *
      system_config_->unified_calib_config.spline_knot_interval;
  const double kOutputEndTime =
      spline_bundle_->GetR3dSpline(trans_spline_info_.name).MaxTime() -
      kOutputMargin;
  double current_time =
      spline_bundle_->GetR3dSpline(trans_spline_info_.name).MinTime() +
      kOutputMargin;

  while (current_time < kOutputEndTime) {
    Sophus::Vector3d trans_W_B;
    Sophus::SO3d rot_W_B;
    spline_bundle_->GetR3dSpline(trans_spline_info_.name)
        .Evaluate(current_time, trans_W_B);
    spline_bundle_->GetSo3dSpline(rot_spline_info_.name)
        .Evaluate(current_time, rot_W_B);
    Eigen::Quaterniond rot_q_W_B = rot_W_B.unit_quaternion();

    traj_file << current_time << " " << trans_W_B.x() << " " << trans_W_B.y()
              << " " << trans_W_B.z() << " " << rot_q_W_B.x() << " "
              << rot_q_W_B.y() << " " << rot_q_W_B.z() << " " << rot_q_W_B.w()
              << std::endl;

    current_time += kOutputInterval;
  }

  return true;
}

bool UnifiedCalibrator::SaveTargetCorners(const std::string& output_path) {
  auto& target_corners_3d = sensor_manager_->GetTargetCorners()->corners;

  // Update the global positions of the target corners with the optimized target
  // poses.
  for (auto& [_, corner] : target_corners_3d) {
    Eigen::Vector3d p_in_Ti = corner.position_local;

    int target_idx = corner.target_idx;
    Eigen::Vector3d trans_W_Ti = calib_parameters_->trans_W_Ti.at(target_idx);
    Sophus::SO3d rot_W_Ti = calib_parameters_->rot_W_Ti.at(target_idx);
    Eigen::Vector3d p_in_W = rot_W_Ti * p_in_Ti + trans_W_Ti;

    corner.position_global = p_in_W;
  }

  // Save the global positions of the target corners to a file.
  std::ofstream corner_file(output_path);
  if (!corner_file.is_open()) {
    spdlog::error("Failed to open file for saving target corners: {}",
                  output_path);
    return false;
  }
  corner_file << std::fixed << std::setprecision(6);

  // The format of each line is [target_idx corner_id x y z].
  for (const auto& [corner_id, corner] : target_corners_3d) {
    corner_file << corner.target_idx << " " << corner_id << " "
                << corner.position_global.x() << " "
                << corner.position_global.y() << " "
                << corner.position_global.z() << std::endl;
  }

  return true;
}

void UnifiedCalibrator::PrintCalibrationResults() {
  std::string body_frame_label = calib_parameters_->body_frame_label;
  std::string base_camera_label = calib_parameters_->base_camera_label;
  int world_target_index = calib_parameters_->world_target_index;
  spdlog::info("Body frame: {}, base camera: {}, world target: {}",
               body_frame_label, base_camera_label, world_target_index);

  // ===========================================================================

  // Step 1: Print camera calibration results.
  spdlog::info(" - Calibration results for base camera:");
  Eigen::Vector3d trans_B_Cb = calib_parameters_->trans_B_Cb;
  Eigen::Quaterniond rot_B_Cb = calib_parameters_->rot_B_Cb.unit_quaternion();
  double toff_B_Cb = calib_parameters_->time_offset_B_Cb;
  spdlog::info("Translation Cb in B: [{:.6f}, {:.6f}, {:.6f}]", trans_B_Cb.x(),
               trans_B_Cb.y(), trans_B_Cb.z());
  spdlog::info(
      "Rotation from Cb to B (quat_xyzw): [{:.6f}, {:.6f}, {:.6f}, {:.6f}]",
      rot_B_Cb.x(), rot_B_Cb.y(), rot_B_Cb.z(), rot_B_Cb.w());
  spdlog::info("Time offset from Cb to B: {:.6f} sec", toff_B_Cb);

  Eigen::Vector3d gravity_in_W = calib_parameters_->gravity_in_W;
  spdlog::info("Gravity direction in world frame W: [{:.6f}, {:.6f}, {:.6f}]",
               gravity_in_W.x(), gravity_in_W.y(), gravity_in_W.z());

  for (const auto& [cam_label, _] : sensor_manager_->GetAllCamSequences()) {
    spdlog::info(" - Calibration results for camera: {}", cam_label);

    Eigen::Vector3d trans_Cb_Ci = calib_parameters_->trans_Cb_Ci.at(cam_label);
    Eigen::Quaterniond rot_Cb_Ci =
        calib_parameters_->rot_Cb_Ci.at(cam_label).unit_quaternion();
    double toff_Cb_Ci = calib_parameters_->time_offset_Cb_Ci.at(cam_label);

    spdlog::info("Translation Ci in Cb: [{:.6f}, {:.6f}, {:.6f}]",
                 trans_Cb_Ci.x(), trans_Cb_Ci.y(), trans_Cb_Ci.z());
    spdlog::info(
        "Rotation from Cb to Ci (quat_xyzw): [{:.6f}, {:.6f}, {:.6f}, "
        "{:.6f}]",
        rot_Cb_Ci.x(), rot_Cb_Ci.y(), rot_Cb_Ci.z(), rot_Cb_Ci.w());
    spdlog::info("Time offset from Ci to Cb: {:.6f} sec", toff_Cb_Ci);
  }

  // ===========================================================================

  // Step 2: Print IMU calibration results.
  for (const auto& [imu_label, _] : sensor_manager_->GetAllImuSequences()) {
    spdlog::info(" - Calibration results for IMU: {}", imu_label);

    Eigen::Vector3d trans_B_Ii = calib_parameters_->trans_B_Ii.at(imu_label);
    Eigen::Quaterniond rot_B_Ii =
        calib_parameters_->rot_B_Ii.at(imu_label).unit_quaternion();
    double toff_B_Ii = calib_parameters_->time_offset_B_Ii.at(imu_label);

    spdlog::info("Translation Ii in B: [{:.6f}, {:.6f}, {:.6f}]",
                 trans_B_Ii.x(), trans_B_Ii.y(), trans_B_Ii.z());
    spdlog::info(
        "Rotation from B to Ii (quat_xyzw): [{:.6f}, {:.6f}, {:.6f}, {:.6f}]",
        rot_B_Ii.x(), rot_B_Ii.y(), rot_B_Ii.z(), rot_B_Ii.w());
    spdlog::info("Time offset from Ii to B: {:.6f} sec", toff_B_Ii);
  }

  // ===========================================================================

  // Step 3: Print magnetometer calibration results.
  for (const auto& [mag_label, _] : sensor_manager_->GetAllMagSequences()) {
    spdlog::info(" - Calibration results for Magnetometer: {}", mag_label);

    Sophus::SO3d rot_B_Mi = calib_parameters_->rot_B_Mi.at(mag_label);
    double toff_B_Mi = calib_parameters_->time_offset_B_Mi.at(mag_label);

    spdlog::info(
        "Rotation from B to Mi (quat_xyzw): [{:.6f}, {:.6f}, {:.6f}, "
        "{:.6f}]",
        rot_B_Mi.unit_quaternion().x(), rot_B_Mi.unit_quaternion().y(),
        rot_B_Mi.unit_quaternion().z(), rot_B_Mi.unit_quaternion().w());
    spdlog::info("Time offset from Mi to B: {:.6f} sec", toff_B_Mi);
  }

  if (!sensor_manager_->GetAllMagSequences().empty()) {
    Eigen::Vector3d mag_in_W = calib_parameters_->mag_in_W;
    spdlog::info(
        "Magnetic field direction in world frame W: [{:.6f}, {:.6f}, {:.6f}]",
        mag_in_W.x(), mag_in_W.y(), mag_in_W.z());
  }

  // ===========================================================================

  // Step 4: Print fiducial target calibration results.
  for (const auto& [target_idx, _] : sensor_manager_->GetAllTargetConfigs()) {
    spdlog::info(" - Calibration results for fiducial target: {}", target_idx);

    Eigen::Vector3d trans_W_Ti = calib_parameters_->trans_W_Ti.at(target_idx);
    Eigen::Quaterniond rot_W_Ti =
        calib_parameters_->rot_W_Ti.at(target_idx).unit_quaternion();

    spdlog::info("Translation Ti in W: [{:.6f}, {:.6f}, {:.6f}]",
                 trans_W_Ti.x(), trans_W_Ti.y(), trans_W_Ti.z());
    spdlog::info(
        "Rotation from Ti to W (quat_xyzw): [{:.6f}, {:.6f}, {:.6f}, {:.6f}]",
        rot_W_Ti.x(), rot_W_Ti.y(), rot_W_Ti.z(), rot_W_Ti.w());
  }

  // ===========================================================================

  return;
}

}  // namespace xr_ucalib
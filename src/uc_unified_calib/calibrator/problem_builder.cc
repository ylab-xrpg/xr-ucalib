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

#include "xr_ucalib/uc_unified_calib/calibrator/problem_builder.h"

#include <cmath>
#include <vector>

#include "xr_ucalib/uc_unified_calib/cost_function/cam_reproj_ct_cost.hpp"
#include "xr_ucalib/uc_unified_calib/cost_function/imu_acc_cost.hpp"
#include "xr_ucalib/uc_unified_calib/cost_function/imu_gyr_cost.hpp"
#include "xr_ucalib/uc_unified_calib/cost_function/mag_cost.hpp"

namespace xr_ucalib {

bool ProblemBuilder::AddCamReprojResiduals(
    ceres::Problem& problem, const std::string& label,
    const CamFrame::Ptr& cam_frame, const TargetCorner3D::Ptr& target_corners,
    const CamConfig& cam_config,
    const std::map<int, TargetConfig>& target_configs) {
  // Step 1: Calculate the meta data for the translational and rotational
  // B-splines corresponding to the measurement.
  SplineMeta<kSplineOrder> trans_meta, rot_meta;

  // Calculate the start and end times of the meta data.
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
        "The min/max times of the spline meta exception for [{}] at {:.9f} ",
        label, cam_frame->timestamp);
    return false;
  }

  spline_bundle->CalculateR3dSplineMeta(
      trans_spline_name, {{meta_min_time, meta_max_time}}, trans_meta);
  spline_bundle->CalculateSo3dSplineMeta(
      rot_spline_name, {{meta_min_time, meta_max_time}}, rot_meta);

  // ===========================================================================

  // Step 2: Create the camera reprojection cost functions and add to the
  // problem.
  // Gather parameter block pointers.
  std::vector<double*> spline_param_blocks;
  // Add the spline knot data to the parameter block vector.
  AddR3dKnotData(spline_bundle->GetR3dSpline(trans_spline_name), trans_meta,
                 problem, spline_param_blocks);
  AddSo3dKnotData(spline_bundle->GetSo3dSpline(rot_spline_name), rot_meta,
                  problem, spline_param_blocks);

  auto trans_B_Cb_ptr = context_.calib_parameters->trans_B_Cb.data();
  auto rot_B_Cb_ptr = context_.calib_parameters->rot_B_Cb.data();
  auto trans_Cb_Ci_ptr =
      context_.calib_parameters->trans_Cb_Ci.at(label).data();
  auto rot_Cb_Ci_ptr = context_.calib_parameters->rot_Cb_Ci.at(label).data();
  auto toff_B_Cb_ptr = &context_.calib_parameters->time_offset_B_Cb;
  auto toff_Cb_Ci_ptr = &context_.calib_parameters->time_offset_Cb_Ci.at(label);
  auto cam_intrinsic_ptr =
      context_.calib_parameters->cam_intrinsics.at(label)->parameters.data();

  // Iterate over all observed keypoints in the current camera frame.
  double weight = 1. / cam_config.noise;
  const auto& camera_intrinsic =
      context_.calib_parameters->cam_intrinsics.at(label);
  std::set<int> involved_target_idx;
  for (const auto& [landmark_id, keypoint] : cam_frame->keypoints) {
    if (!target_corners->Contains(landmark_id)) {
      spdlog::warn("Landmark ID {} not found in target corners.", landmark_id);
      continue;
    }

    // Get the corresponding 3D corner point.
    const auto& corner_3d = target_corners->At(landmark_id).position_local;
    int target_idx = target_corners->At(landmark_id).target_idx;
    if (involved_target_idx.find(target_idx) == involved_target_idx.end()) {
      involved_target_idx.insert(target_idx);
    }

    // Step 2.1: Create a camera reprojection cost function and specify
    // parameter dimensions.
    auto cam_reproj_cost = CamReprojCtCost<kSplineOrder>::Create(
        trans_meta, rot_meta, camera_intrinsic, corner_3d, keypoint,
        cam_frame->timestamp, weight);

    // Parameter blocks for spline knots.
    for (size_t i = 0; i < trans_meta.NumParameters(); ++i) {
      cam_reproj_cost->AddParameterBlock(3);
    }
    for (size_t i = 0; i < rot_meta.NumParameters(); ++i) {
      cam_reproj_cost->AddParameterBlock(4);
    }

    // trans_B_Cb
    cam_reproj_cost->AddParameterBlock(3);
    // rot_B_Cb
    cam_reproj_cost->AddParameterBlock(4);
    // trans_Cb_Ci
    cam_reproj_cost->AddParameterBlock(3);
    // rot_Cb_Ci
    cam_reproj_cost->AddParameterBlock(4);
    // trans_W_Ti
    cam_reproj_cost->AddParameterBlock(3);
    // rot_W_Ti
    cam_reproj_cost->AddParameterBlock(4);
    // toff_B_Cb
    cam_reproj_cost->AddParameterBlock(1);
    // toff_Cb_Ci
    cam_reproj_cost->AddParameterBlock(1);
    // cam_intrinsics
    cam_reproj_cost->AddParameterBlock(camera_intrinsic->parameter_size);
    // Residual dimension
    cam_reproj_cost->SetNumResiduals(2);

    // Step 2.2: Organize the parameter block pointers in a vector.
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

    // Step 2.3: Add the residual block to the problem.
    problem.AddResidualBlock(cam_reproj_cost, huber_loss_function_.get(),
                             param_block_vector);
    // Set the manifolds for rotation parameters.
    problem.SetManifold(rot_B_Cb_ptr, quat_manifold_.get());
    problem.SetManifold(rot_Cb_Ci_ptr, quat_manifold_.get());
    problem.SetManifold(rot_W_Ti_ptr, quat_manifold_.get());
  }

  // ===========================================================================

  // Step 3: Handle case that require fixing calibration parameters.
  // Step 3.1: Camera related parameters.
  // Fix the extrinsics of the base camera.
  if (label == context_.calib_parameters->base_camera_label) {
    problem.SetParameterBlockConstant(trans_Cb_Ci_ptr);
    problem.SetParameterBlockConstant(rot_Cb_Ci_ptr);
    problem.SetParameterBlockConstant(toff_Cb_Ci_ptr);
  }

  if (cam_config.fix_spatial_extrinsic) {
    if (label == context_.calib_parameters->base_camera_label) {
      context_.calib_parameters->trans_B_Cb = cam_config.trans_Cb_Ci_prior;
      context_.calib_parameters->rot_B_Cb =
          Sophus::SO3d(cam_config.rot_q_Cb_Ci_prior);
      problem.SetParameterBlockConstant(trans_B_Cb_ptr);
      problem.SetParameterBlockConstant(rot_B_Cb_ptr);
    } else {
      context_.calib_parameters->trans_Cb_Ci[label] =
          cam_config.trans_Cb_Ci_prior;
      context_.calib_parameters->rot_Cb_Ci[label] =
          Sophus::SO3d(cam_config.rot_q_Cb_Ci_prior);
      problem.SetParameterBlockConstant(trans_Cb_Ci_ptr);
      problem.SetParameterBlockConstant(rot_Cb_Ci_ptr);
    }
  }

  if (cam_config.fix_temporal_extrinsic) {
    if (label == context_.calib_parameters->base_camera_label) {
      context_.calib_parameters->time_offset_B_Cb = cam_config.toff_Cb_Ci_prior;
      problem.SetParameterBlockConstant(toff_B_Cb_ptr);
    } else {
      context_.calib_parameters->time_offset_Cb_Ci.at(label) =
          cam_config.toff_Cb_Ci_prior;
      problem.SetParameterBlockConstant(toff_Cb_Ci_ptr);
    }
  }

  if (cam_config.fix_intrinsic &&
      context_.calib_parameters->cam_intrinsics.at(label)->parameter_size ==
          static_cast<int>(cam_config.intrinsic_prior.size())) {
    context_.calib_parameters->cam_intrinsics.at(label)->parameters =
        cam_config.intrinsic_prior;
    problem.SetParameterBlockConstant(cam_intrinsic_ptr);
  }

  // Fix spatial and temporal extrinsics of non-base cameras.
  // This requires all non-base cameras to be accurately calibrated beforehand.
  if (context_.system_config->unified_calib_config.fix_camera_extrinsics &&
      label != context_.calib_parameters->base_camera_label) {
    problem.SetParameterBlockConstant(trans_Cb_Ci_ptr);
    problem.SetParameterBlockConstant(rot_Cb_Ci_ptr);
    problem.SetParameterBlockConstant(toff_Cb_Ci_ptr);
  }

  if (context_.system_config->unified_calib_config.fix_camera_intrinsics) {
    problem.SetParameterBlockConstant(cam_intrinsic_ptr);
  }

  // Step 3.2: Target related parameters.
  for (const int target_idx : involved_target_idx) {
    auto trans_W_Ti_ptr =
        context_.calib_parameters->trans_W_Ti.at(target_idx).data();
    auto rot_W_Ti_ptr =
        context_.calib_parameters->rot_W_Ti.at(target_idx).data();
    // Fix the world frame target.
    if (target_idx == context_.calib_parameters->world_target_index) {
      problem.SetParameterBlockConstant(trans_W_Ti_ptr);
      problem.SetParameterBlockConstant(rot_W_Ti_ptr);
      continue;
    }

    if (target_configs.at(target_idx).fix_spatial_extrinsic) {
      context_.calib_parameters->trans_W_Ti[target_idx] =
          target_configs.at(target_idx).trans_W_T_prior;
      context_.calib_parameters->rot_W_Ti[target_idx] =
          Sophus::SO3d(target_configs.at(target_idx).rot_q_W_T_prior);
      problem.SetParameterBlockConstant(trans_W_Ti_ptr);
      problem.SetParameterBlockConstant(rot_W_Ti_ptr);
    }
  }

  // ===========================================================================

  return true;
}

bool ProblemBuilder::AddImuAccResiduals(ceres::Problem& problem,
                                        const std::string& label,
                                        const ImuFrame::Ptr& imu_frame,
                                        const ImuConfig& imu_config) {
  // Step 1: Calculate the meta data for the translational and rotational
  // B-splines corresponding to the measurement.
  SplineMeta<kSplineOrder> trans_meta, rot_meta;

  // Calculate the start and end times of the meta data.
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

  // Step 2: Create an accelerometer cost function and specify parameter
  // dimensions.
  double weight =
      1. / (imu_config.noise[0] * std::sqrt(imu_config.frequency_hz));
  auto imu_acc_cost =
      ImuAccCost<kSplineOrder>::Create(trans_meta, rot_meta, imu_frame, weight);

  // Parameter blocks for spline knots.
  for (size_t i = 0; i < trans_meta.NumParameters(); ++i) {
    imu_acc_cost->AddParameterBlock(3);
  }
  for (size_t i = 0; i < rot_meta.NumParameters(); ++i) {
    imu_acc_cost->AddParameterBlock(4);
  }

  // trans_B_Ii
  imu_acc_cost->AddParameterBlock(3);
  // rot_B_Ii
  imu_acc_cost->AddParameterBlock(4);
  // gravity_in_W
  imu_acc_cost->AddParameterBlock(3);
  // toff_B_Ii
  imu_acc_cost->AddParameterBlock(1);
  // acc_bias
  imu_acc_cost->AddParameterBlock(3);
  // acc_scale
  imu_acc_cost->AddParameterBlock(3);
  // acc_non_ortho
  imu_acc_cost->AddParameterBlock(3);
  // Residual dimension
  imu_acc_cost->SetNumResiduals(3);

  // ===========================================================================

  // Step 3: Organize the parameter block pointers in a vector and add the
  // residual block to the problem.
  std::vector<double*> param_block_vector;

  // Add the spline knot data to the parameter block vector.
  AddR3dKnotData(spline_bundle->GetR3dSpline(trans_spline_name), trans_meta,
                 problem, param_block_vector);
  AddSo3dKnotData(spline_bundle->GetSo3dSpline(rot_spline_name), rot_meta,
                  problem, param_block_vector);

  // Add the calibration parameters to the vector.
  auto trans_B_Ii_ptr = context_.calib_parameters->trans_B_Ii.at(label).data();
  auto rot_B_Ii_ptr = context_.calib_parameters->rot_B_Ii.at(label).data();
  auto gravity_in_W = context_.calib_parameters->gravity_in_W.data();
  auto toff_B_Ii_ptr = &context_.calib_parameters->time_offset_B_Ii.at(label);
  auto acc_bias_ptr =
      context_.calib_parameters->imu_intrinsics.at(label)->acc_bias.data();
  auto acc_scale_ptr =
      context_.calib_parameters->imu_intrinsics.at(label)->acc_scale.data();
  auto acc_non_ortho_ptr = context_.calib_parameters->imu_intrinsics.at(label)
                               ->acc_non_orthogonal.data();

  param_block_vector.push_back(trans_B_Ii_ptr);
  param_block_vector.push_back(rot_B_Ii_ptr);
  param_block_vector.push_back(gravity_in_W);
  param_block_vector.push_back(toff_B_Ii_ptr);
  param_block_vector.push_back(acc_bias_ptr);
  param_block_vector.push_back(acc_scale_ptr);
  param_block_vector.push_back(acc_non_ortho_ptr);

  // Add the residual block to the problem.
  problem.AddResidualBlock(imu_acc_cost, huber_loss_function_.get(),
                           param_block_vector);
  // Set the manifolds for rotation and gravity parameters.
  problem.SetManifold(rot_B_Ii_ptr, quat_manifold_.get());
  problem.SetManifold(gravity_in_W, sphere_manifold_.get());

  // ===========================================================================

  // Step 4: Handle case that require fixing calibration parameters.
  if (label == context_.calib_parameters->body_frame_label) {
    problem.SetParameterBlockConstant(trans_B_Ii_ptr);
    problem.SetParameterBlockConstant(rot_B_Ii_ptr);
    problem.SetParameterBlockConstant(toff_B_Ii_ptr);
  }

  if (imu_config.fix_spatial_extrinsic &&
      label != context_.calib_parameters->body_frame_label) {
    context_.calib_parameters->trans_B_Ii.at(label) =
        imu_config.trans_B_Ii_prior;
    context_.calib_parameters->rot_B_Ii.at(label) =
        Sophus::SO3d(imu_config.rot_q_B_Ii_prior);
    problem.SetParameterBlockConstant(trans_B_Ii_ptr);
    problem.SetParameterBlockConstant(rot_B_Ii_ptr);
  }

  if (imu_config.fix_temporal_extrinsic &&
      label != context_.calib_parameters->body_frame_label) {
    context_.calib_parameters->time_offset_B_Ii.at(label) =
        imu_config.toff_B_Ii_prior;
    problem.SetParameterBlockConstant(toff_B_Ii_ptr);
  }

  if (imu_config.imu_model_type == ImuModelType::CALIBRATED ||
      imu_config.imu_model_type == ImuModelType::MISALIGN) {
    problem.SetParameterBlockConstant(acc_scale_ptr);
    problem.SetParameterBlockConstant(acc_non_ortho_ptr);
  }

  // ===========================================================================

  return true;
}

bool ProblemBuilder::AddImuGyrResiduals(ceres::Problem& problem,
                                        const std::string& label,
                                        const ImuFrame::Ptr& imu_frame,
                                        const ImuConfig& imu_config) {
  // Step 1: Calculate the meta data for the rotational B-spline corresponding
  // to the measurement.
  SplineMeta<kSplineOrder> rot_meta;

  // Calculate the start and end times of the meta data.
  double meta_min_time = -1.;
  double meta_max_time = -1.;
  const double kMeasTimestamp = imu_frame->timestamp;

  if (!CalMetaMinMaxTime(kMeasTimestamp, meta_min_time, meta_max_time)) {
    return false;
  }

  const auto& spline_bundle = context_.spline_bundle;
  const auto& rot_spline_name = context_.rot_spline_info.name;
  if (!spline_bundle->TimeInRangeForSo3dSpline(meta_min_time,
                                               rot_spline_name) ||
      !spline_bundle->TimeInRangeForSo3dSpline(meta_max_time,
                                               rot_spline_name)) {
    spdlog::critical(
        "The min/max times of the spline meta exception for [{}] at {:.9f} ",
        label, imu_frame->timestamp);
    return false;
  }

  spline_bundle->CalculateSo3dSplineMeta(
      rot_spline_name, {{meta_min_time, meta_max_time}}, rot_meta);

  // ===========================================================================

  // Step 2: Create a gyroscope cost function and specify parameter
  // dimensions.
  double weight =
      1. / (imu_config.noise[2] * std::sqrt(imu_config.frequency_hz));
  auto imu_gyr_cost =
      ImuGyrCost<kSplineOrder>::Create(rot_meta, imu_frame, weight);

  // Parameter blocks for spline knots.
  for (size_t i = 0; i < rot_meta.NumParameters(); ++i) {
    imu_gyr_cost->AddParameterBlock(4);
  }

  // rot_B_Ii
  imu_gyr_cost->AddParameterBlock(4);
  // toff_B_Ii
  imu_gyr_cost->AddParameterBlock(1);
  // gyr_bias
  imu_gyr_cost->AddParameterBlock(3);
  // gyr_scale
  imu_gyr_cost->AddParameterBlock(3);
  // gyr_non_ortho
  imu_gyr_cost->AddParameterBlock(3);
  // rot_gyr_acc
  imu_gyr_cost->AddParameterBlock(4);
  // Residual dimension
  imu_gyr_cost->SetNumResiduals(3);

  // ===========================================================================

  // Step 3: Organize the parameter block pointers in a vector and add the
  // residual block to the problem.
  std::vector<double*> param_block_vector;

  // Add the spline knot data to the parameter block vector.
  AddSo3dKnotData(spline_bundle->GetSo3dSpline(rot_spline_name), rot_meta,
                  problem, param_block_vector);

  // Add the calibration parameters to the vector.
  auto rot_B_Ii_ptr = context_.calib_parameters->rot_B_Ii.at(label).data();
  auto toff_B_Ii_ptr = &context_.calib_parameters->time_offset_B_Ii.at(label);
  auto gyr_bias_ptr =
      context_.calib_parameters->imu_intrinsics.at(label)->gyr_bias.data();
  auto gyr_scale_ptr =
      context_.calib_parameters->imu_intrinsics.at(label)->gyr_scale.data();
  auto gyr_non_ortho_ptr = context_.calib_parameters->imu_intrinsics.at(label)
                               ->gyr_non_orthogonal.data();
  auto rot_gyr_acc_ptr =
      context_.calib_parameters->imu_intrinsics.at(label)->rot_gyr_acc.data();

  param_block_vector.push_back(rot_B_Ii_ptr);
  param_block_vector.push_back(toff_B_Ii_ptr);
  param_block_vector.push_back(gyr_bias_ptr);
  param_block_vector.push_back(gyr_scale_ptr);
  param_block_vector.push_back(gyr_non_ortho_ptr);
  param_block_vector.push_back(rot_gyr_acc_ptr);

  // Add the residual block to the problem.
  problem.AddResidualBlock(imu_gyr_cost, nullptr, param_block_vector);
  // Set the manifolds for rotation parameters.
  problem.SetManifold(rot_B_Ii_ptr, quat_manifold_.get());
  problem.SetManifold(rot_gyr_acc_ptr, quat_manifold_.get());

  // ===========================================================================

  // Step 4: Handle case that require fixing calibration parameters.
  if (label == context_.calib_parameters->body_frame_label) {
    problem.SetParameterBlockConstant(rot_B_Ii_ptr);
    problem.SetParameterBlockConstant(toff_B_Ii_ptr);
  }

  if (imu_config.fix_spatial_extrinsic &&
      label != context_.calib_parameters->body_frame_label) {
    context_.calib_parameters->rot_B_Ii.at(label) =
        Sophus::SO3d(imu_config.rot_q_B_Ii_prior);
    problem.SetParameterBlockConstant(rot_B_Ii_ptr);
  }

  if (imu_config.fix_temporal_extrinsic &&
      label != context_.calib_parameters->body_frame_label) {
    context_.calib_parameters->time_offset_B_Ii.at(label) =
        imu_config.toff_B_Ii_prior;
    problem.SetParameterBlockConstant(toff_B_Ii_ptr);
  }

  if (imu_config.imu_model_type == ImuModelType::CALIBRATED) {
    problem.SetParameterBlockConstant(gyr_scale_ptr);
    problem.SetParameterBlockConstant(gyr_non_ortho_ptr);
    problem.SetParameterBlockConstant(rot_gyr_acc_ptr);
  } else if (imu_config.imu_model_type == ImuModelType::SCALE) {
    problem.SetParameterBlockConstant(rot_gyr_acc_ptr);
  } else if (imu_config.imu_model_type == ImuModelType::MISALIGN) {
    problem.SetParameterBlockConstant(gyr_scale_ptr);
    problem.SetParameterBlockConstant(gyr_non_ortho_ptr);
  }

  // ===========================================================================

  return true;
}

bool ProblemBuilder::AddMagResiduals(ceres::Problem& problem,
                                     const std::string& label,
                                     const MagFrame::Ptr& mag_frame,
                                     const MagConfig& mag_config) {
  // Step 1: Calculate the meta data for the rotational B-spline corresponding
  // to the measurement.
  SplineMeta<kSplineOrder> rot_meta;

  // Calculate the start and end times of the meta data.
  double meta_min_time = -1.;
  double meta_max_time = -1.;
  const double kMeasTimestamp = mag_frame->timestamp;

  if (!CalMetaMinMaxTime(kMeasTimestamp, meta_min_time, meta_max_time)) {
    return false;
  }

  const auto& spline_bundle = context_.spline_bundle;
  const auto& rot_spline_name = context_.rot_spline_info.name;
  if (!spline_bundle->TimeInRangeForSo3dSpline(meta_min_time,
                                               rot_spline_name) ||
      !spline_bundle->TimeInRangeForSo3dSpline(meta_max_time,
                                               rot_spline_name)) {
    spdlog::critical(
        "The min/max times of the spline meta exception for [{}] at {:.9f} ",
        label, mag_frame->timestamp);
    return false;
  }

  spline_bundle->CalculateSo3dSplineMeta(
      rot_spline_name, {{meta_min_time, meta_max_time}}, rot_meta);

  // ===========================================================================

  // Step 2: Create a magnetometer cost function and specify parameter
  // dimensions.
  double weight = 1. / mag_config.noise;
  auto mag_cost = MagCost<kSplineOrder>::Create(rot_meta, mag_frame, weight);

  // Parameter blocks for spline knots.
  for (size_t i = 0; i < rot_meta.NumParameters(); ++i) {
    mag_cost->AddParameterBlock(4);
  }

  // rot_B_Mi
  mag_cost->AddParameterBlock(4);
  // mag_in_W
  mag_cost->AddParameterBlock(3);
  // toff_B_Mi
  mag_cost->AddParameterBlock(1);
  // Residual dimension
  mag_cost->SetNumResiduals(3);

  // ===========================================================================

  // Step 3: Organize the parameter block pointers in a vector and add the
  // residual block to the problem.
  std::vector<double*> param_block_vector;

  // Add the spline knot data to the parameter block vector.
  AddSo3dKnotData(spline_bundle->GetSo3dSpline(rot_spline_name), rot_meta,
                  problem, param_block_vector);

  // Add the calibration parameters to the vector.
  auto rot_B_Mi_ptr = context_.calib_parameters->rot_B_Mi.at(label).data();
  auto mag_in_W_ptr = context_.calib_parameters->mag_in_W.data();
  auto toff_B_Mi_ptr = &context_.calib_parameters->time_offset_B_Mi.at(label);

  param_block_vector.push_back(rot_B_Mi_ptr);
  param_block_vector.push_back(mag_in_W_ptr);
  param_block_vector.push_back(toff_B_Mi_ptr);

  // Add the residual block to the problem.
  problem.AddResidualBlock(mag_cost, nullptr, param_block_vector);
  // Set the manifolds for rotation parameters.
  problem.SetManifold(rot_B_Mi_ptr, quat_manifold_.get());
  problem.SetManifold(mag_in_W_ptr, sphere_manifold_.get());

  // ===========================================================================

  // Step 4: Handle case that require fixing calibration parameters.
  if (mag_config.fix_spatial_extrinsic) {
    context_.calib_parameters->rot_B_Mi.at(label) =
        Sophus::SO3d(mag_config.rot_q_B_Mi_prior);
    problem.SetParameterBlockConstant(rot_B_Mi_ptr);
  }

  if (mag_config.fix_temporal_extrinsic) {
    context_.calib_parameters->time_offset_B_Mi.at(label) =
        mag_config.toff_B_Mi_prior;
    problem.SetParameterBlockConstant(toff_B_Mi_ptr);
  }

  // ===========================================================================

  return true;
}

bool ProblemBuilder::CalMetaMinMaxTime(const double& meas_time,
                                       double& min_time, double& max_time) {
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

void ProblemBuilder::AddR3dKnotData(
    const SplineBundleType::R3dSplineType& spline,
    const SplineMetaType& spline_meta, ceres::Problem& problem,
    std::vector<double*>& param_block_vector) {
  for (const auto& segment : spline_meta.segments) {
    // Compute time index, 'knot_interval * 0.5' is the treatment for
    // numerical accuracy
    size_t index;
    double fraction;
    spline.ComputeTimeIndex(segment.start_time + segment.knot_interval * 0.5,
                            index, fraction);

    // Iterate over control points in the segment.
    for (std::size_t i = index; i < index + segment.NumParameters(); ++i) {
      auto* data =
          const_cast<double*>(spline.get_knot(static_cast<int>(i)).data());

      // Add the control point as a parameter block.
      if (!problem.HasParameterBlock(data)) {
        problem.AddParameterBlock(data, 3);
      }

      // Store the control point data in the parameter block vector.
      param_block_vector.push_back(data);
    }
  }
}

void ProblemBuilder::AddSo3dKnotData(
    const SplineBundleType::So3dSplineType& spline,
    const SplineMetaType& spline_meta, ceres::Problem& problem,
    std::vector<double*>& param_block_vector) {
  for (const auto& segment : spline_meta.segments) {
    // Compute time index, 'knot_interval * 0.5' is the treatment for
    // numerical accuracy
    size_t index;
    double fraction;
    spline.ComputeTimeIndex(segment.start_time + segment.knot_interval * 0.5,
                            index, fraction);

    // Iterate over control points in the segment.
    for (std::size_t i = index; i < index + segment.NumParameters(); ++i) {
      auto* data =
          const_cast<double*>(spline.get_knot(static_cast<int>(i)).data());

      // Add the control point as a parameter block.
      if (!problem.HasParameterBlock(data)) {
        problem.AddParameterBlock(data, 4);
        problem.SetManifold(data, quat_manifold_.get());
      }

      // Store the control point data in the parameter block vector.
      param_block_vector.push_back(data);
    }
  }
}

}  // namespace xr_ucalib
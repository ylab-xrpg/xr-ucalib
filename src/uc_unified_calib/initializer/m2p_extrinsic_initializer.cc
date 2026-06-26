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

#include "xr_ucalib/uc_unified_calib/initializer/m2p_extrinsic_initializer.h"

#include <spdlog/spdlog.h>

namespace xr_ucalib {

bool M2PExtrinsicInitializer::EstimateFromSeq(const PoseSequence::Ptr pose_seq,
                                              const MagSequence::Ptr mag_seq,
                                              const double &toff_P_M,
                                              Eigen::Quaterniond &rot_q_P_M,
                                              Eigen::Vector3d &mag_field_in_W) {
  // Set default values.
  rot_q_P_M = Eigen::Quaterniond::Identity();
  mag_field_in_W = Eigen::Vector3d::Zero();

  if (pose_seq->Empty() || mag_seq->Empty()) {
    spdlog::error("Empty pose or mag sequence in M2P initialization.");
    return false;
  }

  // ==========================================================================

  // Step 1: Set joint time range (under the clock of the pose sequence).
  double pose_start_time = pose_seq->Front()->timestamp + time_margin_;
  double pose_end_time = pose_seq->Back()->timestamp - time_margin_;
  double mag_start_time = mag_seq->Front()->timestamp + toff_P_M + time_margin_;
  double mag_end_time = mag_seq->Back()->timestamp + toff_P_M - time_margin_;

  double joint_start_time = std::max(pose_start_time, mag_start_time);
  double joint_end_time = std::min(pose_end_time, mag_end_time);
  if (joint_start_time >= joint_end_time) {
    spdlog::error("Invalid time overlap in M2P initialization: {:.9f} / {:.9f}",
                  joint_start_time, joint_end_time);
    return false;
  }

  // ===========================================================================

  // Step 2: Construct solver elements.
  M2PSolverElements solver_elements;
  solver_elements.reserve(max_element_num_);

  PoseFrame::Ptr ref_pose = nullptr;
  MagFrame::Ptr ref_mag = nullptr;
  for (const auto &pose_frame : pose_seq->GetFrames()) {
    double curr_pose_time = pose_frame->timestamp;

    if (curr_pose_time < joint_start_time || curr_pose_time > joint_end_time) {
      continue;
    }

    // Find and interpolate Mag data.
    double target_mag_time = curr_pose_time - toff_P_M;
    auto it =
        std::upper_bound(mag_seq->begin(), mag_seq->end(), target_mag_time,
                         [](double time, const MagFrame::Ptr &frame) {
                           return time < frame->timestamp;
                         });

    if (it == mag_seq->begin() || it == mag_seq->end()) {
      continue;
    }

    auto mag_prev = *(it - 1);
    auto mag_next = *it;
    MagFrame::Ptr curr_mag = MagFrame::Create();
    LinInterpMagFrame(mag_prev, mag_next, target_mag_time, curr_mag);

    // Initialization of first keyframe
    if (!ref_pose) {
      ref_pose = pose_frame;
      ref_mag = curr_mag;
      continue;
    }

    // Check rotation change relative to ref_pose
    Eigen::Quaterniond q_W_Pi = ref_pose->rot_q;
    Eigen::Quaterniond q_W_Pj = pose_frame->rot_q;
    Eigen::AngleAxisd rot_diff(q_W_Pi.inverse() * q_W_Pj);

    if (std::abs(rot_diff.angle()) < element_rot_thresh_) {
      continue;
    }

    // Found a new keyframe j. Create element (i->j).
    solver_elements.push_back(M2PSolverElement::Create(
        ref_mag->mag, curr_mag->mag, q_W_Pi.inverse() * q_W_Pj, q_W_Pj));

    if (solver_elements.size() >= static_cast<size_t>(max_element_num_)) break;

    // Update reference frame.
    ref_pose = pose_frame;
    ref_mag = curr_mag;
  }

  // ==========================================================================

  // Step 3: Solve for rotation and magnetic field.
  if (!EstimateRot(solver_elements, rot_q_P_M)) {
    spdlog::error("Failed to estimate rotation in M2P initialization.");
    return false;
  }

  EstimateMagField(solver_elements, rot_q_P_M, mag_field_in_W);

  // ==========================================================================

  return true;
}

bool M2PExtrinsicInitializer::EstimateRot(
    const M2PSolverElements &solver_elements, Eigen::Quaterniond &rot_q_P_M) {
  int num_elements = solver_elements.size();

  if (num_elements < min_element_num_) {
    spdlog::error(
        "Insufficient solver elements ({}) in M2P initialization. Please "
        "extend data duration and increase motion stimuli. ",
        num_elements);
    return false;
  }

  // Each element is a constraint pair.
  Eigen::MatrixXd A(3 * num_elements, 9);
  Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

  for (int i = 0; i < num_elements; ++i) {
    const auto &element = solver_elements[i];
    const Eigen::Vector3d &v_i = element->mag_i;
    const Eigen::Vector3d &v_j = element->mag_j;

    // Delta rotation: rot_q_Pi_Pj = R_Pi^T * R_Pj.
    Eigen::Matrix3d dR = element->rot_q_Pi_Pj.toRotationMatrix();

    // Constraint: [ (v_i^T (x) I) - (v_j^T (x) dR) ] vec(R_ext) = 0, where (x)
    // is the kronecker product.
    //  block construction
    for (int k = 0; k < 3; ++k) {
      // v_i(k)*I - v_j(k)*dR
      A.block<3, 3>(3 * i, 3 * k) = v_i(k) * I3 - v_j(k) * dR;
    }
  }

  // Solve SVD
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);

  Eigen::VectorXd x = svd.matrixV().col(8);

  Eigen::Matrix3d R_est = Eigen::Map<const Eigen::Matrix3d>(x.data());

  // Project to SO(3)
  Eigen::JacobiSVD<Eigen::Matrix3d> svd_R(
      R_est, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R_final = svd_R.matrixU() * svd_R.matrixV().transpose();

  if (R_final.determinant() < 0) {
    R_final = -R_final;
  }

  rot_q_P_M = Eigen::Quaterniond(R_final);
  rot_q_P_M.normalize();

  return true;
}

void M2PExtrinsicInitializer::EstimateMagField(
    const M2PSolverElements &solver_elements,
    const Eigen::Quaterniond &rot_q_P_M, Eigen::Vector3d &mag_field_in_W) {
  Eigen::Vector3d sum_mag = Eigen::Vector3d::Zero();
  for (const auto &element : solver_elements) {
    // m_W = R_WP * R_PM * m_M
    // We used R_WP_j stored in the element.
    sum_mag += element->rot_q_W_Pj * rot_q_P_M * element->mag_j;
  }
  mag_field_in_W = sum_mag / static_cast<double>(solver_elements.size());

  // Verification: check dispersion
  double sum_sq_error = 0.0;
  for (const auto &element : solver_elements) {
    Eigen::Vector3d m_in_W = element->rot_q_W_Pj * rot_q_P_M * element->mag_j;
    sum_sq_error += (m_in_W - mag_field_in_W).squaredNorm();
  }
  double rmse = std::sqrt(sum_sq_error / solver_elements.size());

  if (rmse > mag_verification_thresh_) {
    spdlog::error(
        "M2P Extrinsic Initialization Result is too dispersed (RMSE={:.4f} > "
        "{:.4f}). Calibration likely failed.",
        rmse, mag_verification_thresh_);
  } else if (rmse > mag_verification_thresh_ * 0.5) {
    spdlog::warn(
        "M2P Extrinsic Initialization Result is dispersed (RMSE={:.4f} > "
        "{:.4f}). Result might be unreliable.",
        rmse, mag_verification_thresh_ * 0.5);
  }

  mag_field_in_W.normalize();
}

void M2PExtrinsicInitializer::LinInterpMagFrame(const MagFrame::Ptr &data_0,
                                                const MagFrame::Ptr &data_1,
                                                const double &timestamp,
                                                MagFrame::Ptr &data_result) {
  double lambda =
      (timestamp - data_0->timestamp) / (data_1->timestamp - data_0->timestamp);

  data_result->timestamp = timestamp;
  data_result->mag = (1 - lambda) * data_0->mag + lambda * data_1->mag;
}

}  // namespace xr_ucalib

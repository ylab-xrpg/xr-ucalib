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

#include <memory>
#include <vector>

#include "xr_ucalib/uc_common/sensor_data/sensor_manager.h"

namespace xr_ucalib {

/**
 * @brief Elements for solving extrinsic parameters from Magnetometer to Pose.
 *
 * Contains relative constraints between magnetometer and pose measurements for
 * a time interval [i, j].
 */
struct M2PSolverElement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<M2PSolverElement>;

  /**
   * @brief Construct a M2P extrinsic parameter solver element.
   *
   * @param[in] mag_i Measured magnetic field at start time i.
   * @param[in] mag_j Measured magnetic field at end time j.
   * @param[in] q_Pi_Pj Relative rotation from i to j (q_Pi^{-1} * q_Pj).
   * @param[in] q_W_Pj Absolute rotation of frame j (R_W_Pj), used for field
   * recovery.
   */
  M2PSolverElement(const Eigen::Vector3d &mag_i, const Eigen::Vector3d &mag_j,
                   const Eigen::Quaterniond &q_Pi_Pj,
                   const Eigen::Quaterniond &q_W_Pj)
      : mag_i(mag_i), mag_j(mag_j), rot_q_Pi_Pj(q_Pi_Pj), rot_q_W_Pj(q_W_Pj) {}

  static M2PSolverElement::Ptr Create(const Eigen::Vector3d &mag_i,
                                      const Eigen::Vector3d &mag_j,
                                      const Eigen::Quaterniond &q_Pi_Pj,
                                      const Eigen::Quaterniond &q_W_Pj) {
    return Ptr(new M2PSolverElement(mag_i, mag_j, q_Pi_Pj, q_W_Pj));
  }

  // Magnetometer measurements at time i and j
  Eigen::Vector3d mag_i;
  Eigen::Vector3d mag_j;
  // Pose rotations between i and j, and at j.
  Eigen::Quaterniond rot_q_Pi_Pj;
  Eigen::Quaterniond rot_q_W_Pj;
};

/**
 * @brief Class for initializing extrinsic parameters between Pose and
 * Magnetometer sequences.
 */
class M2PExtrinsicInitializer {
 public:
  using Ptr = std::shared_ptr<M2PExtrinsicInitializer>;

  using M2PSolverElements = std::vector<M2PSolverElement::Ptr>;

  // Create an instance of M2P extrinsic initializer.
  static M2PExtrinsicInitializer::Ptr Create() {
    return Ptr(new M2PExtrinsicInitializer);
  }

  /**
   * @brief Estimate initial guesses of extrinsic rotation between pose and
   * Magnetometer.
   *
   * We perform linear least squares optimization based on the motion
   * constraints between magnetometer and pose measurements.
   *
   * @param[in] pose_seq Pose sequence.
   * @param[in] mag_seq Magnetometer sequence.
   * @param[in] toff_P_M Time offset between pose and Mag.
   * @param[out] rot_q_P_M Initial guess of the extrinsic rotation (Pose T Mag,
   * or Mag in Pose). Corresponds to R_PM (Rotation that takes vector from Mag
   * frame to Pose frame).
   * @param[out] mag_field_in_W Estimated magnetic field vector in World frame.
   * @return True if successful.
   */
  bool EstimateFromSeq(const PoseSequence::Ptr pose_seq,
                       const MagSequence::Ptr mag_seq, const double &toff_P_M,
                       Eigen::Quaterniond &rot_q_P_M,
                       Eigen::Vector3d &mag_field_in_W);

  // Initializer settings setters
  void set_time_margin(const double &v) { time_margin_ = v; }

  void set_rot_diff_thresh(const double &v) { element_rot_thresh_ = v; }

  void set_min_element_num(const int &v) { min_element_num_ = v; }

  void set_max_element_num(const int &v) { max_element_num_ = v; }

  void set_mag_verification_thresh(const double &v) {
    mag_verification_thresh_ = v;
  }

 private:
  M2PExtrinsicInitializer() = default;

  /**
   * @brief Estimate Extrinsic Rotation using SVD on constructed linear system.
   *
   * @param[in] solver_elements List of synced measurements.
   * @param[out] rot_q_P_M Estimated rotation.
   * @return True if successful.
   */
  bool EstimateRot(const M2PSolverElements &solver_elements,
                   Eigen::Quaterniond &rot_q_P_M);

  /**
   * @brief Estimate Magnetic Field in World frame based on estimated
   * extrinsics.
   *
   * @param[in] solver_elements List of synced measurements.
   * @param[in] rot_q_P_M Estimated rotation.
   * @param[out] mag_field_in_W Estimated magnetic field.
   */
  void EstimateMagField(const M2PSolverElements &solver_elements,
                        const Eigen::Quaterniond &rot_q_P_M,
                        Eigen::Vector3d &mag_field_in_W);

  /**
   * @brief Linear interpolation of MagFrame to target timestamp.
   *
   * @param[in] data_0 First MagFrame.
   * @param[in] data_1 Second MagFrame.
   * @param[in] timestamp Target timestamp for interpolation.
   * @param[out] data_result Resulting interpolated MagFrame.
   */
  static void LinInterpMagFrame(const MagFrame::Ptr &data_0,
                                const MagFrame::Ptr &data_1,
                                const double &timestamp,
                                MagFrame::Ptr &data_result);

  // Default initializer settings.
  double time_margin_ = 0.05;
  double element_rot_thresh_ =
      0.1;  // Minimum rotation angle (radians) to trigger new element creation.
  int min_element_num_ = 30;
  int max_element_num_ = 300;
  double mag_verification_thresh_ =
      0.2;  // Threshold for magnetic field dispersion.
};

}  // namespace xr_ucalib

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
#include "xr_ucalib/uc_unified_calib/initializer/imu_preintegrator.h"
#include "xr_ucalib/uc_unified_calib/initializer/initializer_utils.hpp"

namespace xr_ucalib {
/**
 * @brief Elements for solving spatial extrinsic parameters from IMU to pose.
 *
 * Each element contains the pose and IMU preintegration results for a time
 * interval [i, j].
 */
struct I2PSolverElement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<I2PSolverElement>;

  /**
   * @brief Construct a I2P extrinsic parameter solver element.
   *
   * @param[in] integrator IMU preintegration result over a time interval.
   * @param[in] p_i Translational part of the pose measurement at the start
   * time i.
   * @param[in] q_i Rotational part of the pose measurement at the start
   * time i.
   * @param[in] p_j Translational part of the pose measurement at the end
   * time j.
   * @param[in] q_j Rotational part of the pose measurement at the end time j.
   * @param[in] interval Time interval.
   * @param[in] high_quality Flag indicating whether the element is of high
   * quality.
   */
  I2PSolverElement(const std::shared_ptr<ImuPreintegrator> &integrator,
                   const Eigen::Vector3d &p_i, const Eigen::Quaterniond &q_i,
                   const Eigen::Vector3d &p_j, const Eigen::Quaterniond &q_j,
                   const double &interval, const bool &high_quality = false)
      : preintegrator(integrator),
        trans_W_P_i(p_i),
        rot_q_W_P_i(q_i),
        trans_W_P_j(p_j),
        rot_q_W_P_j(q_j),
        time_interval(interval),
        high_quality_flag(high_quality) {}

  // Create an instance of I2P solver element.
  static I2PSolverElement::Ptr Create(
      const std::shared_ptr<ImuPreintegrator> &integrator,
      const Eigen::Vector3d &p_i, const Eigen::Quaterniond &q_i,
      const Eigen::Vector3d &p_j, const Eigen::Quaterniond &q_j,
      const double &interval, const bool &high_quality = false) {
    return Ptr(new I2PSolverElement(integrator, p_i, q_i, p_j, q_j, interval,
                                    high_quality));
  }

  // IMU preintegration result.
  std::shared_ptr<ImuPreintegrator> preintegrator;
  // Pose measurement at the start i and end time j.
  Eigen::Vector3d trans_W_P_i;
  Eigen::Quaterniond rot_q_W_P_i;
  Eigen::Vector3d trans_W_P_j;
  Eigen::Quaterniond rot_q_W_P_j;
  // Length of the time interval.
  double time_interval;
  // Flag for high quality element.
  bool high_quality_flag;
};

/**
 * @brief Class for initializing spatial extrinsic parameters between pose and
 * IMU sequences.
 */
class I2PExtrinsicInitializer {
 public:
  using Ptr = std::shared_ptr<I2PExtrinsicInitializer>;

  using I2PSolverElements = std::vector<I2PSolverElement::Ptr>;

  // Create an instance of I2P extrinsic parameters initializer.
  static I2PExtrinsicInitializer::Ptr Create() {
    return Ptr(new I2PExtrinsicInitializer);
  }

  /**
   * @brief Estimate initial guesses of spatial extrinsic parameters between
   * pose and IMU data sequence.
   *
   * We perform linear least squares optimization based on the motion
   * constraints between IMU preintegration and relative pose.
   * Here, (P) denotes the body frame of the pose sequence, (W) denotes the
   * world, and (I) denotes the body of IMU.
   *
   * @param[in] pose_seq Pose sequence.
   * @param[in] imu_seq IMU sequence.
   * @param[in] toff_P_I Time offset between pose and IMU (toff_P_I).
   * @param[in] g_magnitude Gravity magnitude.
   * @param[out] trans_P_I Initial guess of the extrinsic translation (I in P).
   * @param[out] rot_q_P_I Initial guess of the extrinsic rotation (I to P).
   * @param[out] gravity_in_W Initial guess of the gravity vector represented in
   * world frame.
   * @return True if we are able to initialize the extrinsic parameters.
   */
  bool EstimateFromSeq(const PoseSequence::Ptr pose_seq,
                       const ImuSequence::Ptr imu_seq, const double &toff_P_I,
                       const double &g_magnitude, Eigen::Vector3d &trans_P_I,
                       Eigen::Quaterniond &rot_q_P_I,
                       Eigen::Vector3d &gravity_in_W);

  // Set the parameters.
  void set_time_margin(const double &v) { time_margin_ = v; }

  void set_element_interval_thresh(const double &v) {
    element_interval_thresh_ = v;
  }

  void set_element_trans_thresh(const double &v) { element_trans_thresh_ = v; }

  void set_element_rot_thresh(const double &v) { element_rot_thresh_ = v; }

  void set_min_element_num(const int &v) { min_element_num_ = v; }

  void set_max_element_num(const int &v) { max_element_num_ = v; }

  void set_robust_kernel_coeff(const int &v) { robust_kernel_coeff_ = v; }

  void set_refine_iter_num(const int &v) { refine_iter_num_ = v; }

 private:
  /**
   * @brief Retrieve IMU data from an IMU sequence for a given time period and
   * calculate the preintegration result.
   *
   * @param[in] imu_seq IMU sequence.
   * @param[in] time_0 Start time of preintegration.
   * @param[in] time_1 End time of preintegration.
   * @param[in] g_magnitude Gravity magnitude.
   * @param[out] integrator Preintegration result.
   * @return True if preintegration is completed.
   */
  bool Preintegrate(const ImuSequence::Ptr imu_seq, const double &time_0,
                    const double &time_1, const double &g_magnitude,
                    ImuPreintegrator::Ptr &integrator);

  /**
   * @brief Linear interpolation between two IMU frames to get the result with
   * target timestamp.
   *
   * @param[in] data_0 First IMU frame.
   * @param[in] data_1 Second IMU frame.
   * @param[in] timestamp Target timestamp.
   * @param[out] data_result Result IMU with target timestamp.
   */
  static void LinInterpImuFrame(const ImuFrame::Ptr &data_0,
                                const ImuFrame::Ptr &data_1,
                                const double &timestamp,
                                ImuFrame::Ptr &data_result);

  /**
   * @brief Estimating the initial guess of the extrinsic rotation.
   *
   * @param[in] solver_elements Solver elements for constructing the linear
   * solving system.
   * @param[out] rot_q_P_I Initial guess of the extrinsic rotation (I to P).
   */
  void EstimateRot(const I2PSolverElements &solver_elements,
                   Eigen::Quaterniond &rot_q_P_I);

  /**
   * @brief Simultaneously estimate the initial guesses of the extrinsic
   * translation (I in P) and the gravity vector represented in W.
   *
   * @param[in] solver_elements Solver elements for constructing the linear
   * solving system.
   * @param[in] rot_q_P_I The solved extrinsic rotation (I to P).
   * @param[out] trans_P_I Initial guess of the extrinsic translation (I in P).
   * @param[out] gravity_in_W Initial guess of the gravity vector represented in
   * W.
   */
  void EstimateTransGravityAlign(const I2PSolverElements &solver_elements,
                                 const Eigen::Quaterniond &rot_q_P_I,
                                 Eigen::Vector3d &trans_P_I,
                                 Eigen::Vector3d &gravity_in_W);

  /**
   * @brief Iteratively refine the initial guesses of the extrinsic translation
   * (I in P) and the gravity vector represented in W using the constraint of
   * gravity magnitude.
   *
   * @param[in] solver_elements Solver elements for constructing the linear
   * solving system.
   * @param[in] rot_q_P_I The solved extrinsic rotation (I to P).
   * @param[in] gravity_magnitude Gravity magnitude.
   * @param[out] trans_P_I Initial guess of the extrinsic translation (I in P).
   * @param[out] gravity_in_W Initial guess of the gravity vector represented in
   * W, whose magnitude is strictly constrained to gravity_magnitude.
   */
  void RefineGravityAlign(const I2PSolverElements &solver_elements,
                          const Eigen::Quaterniond &rot_q_P_I,
                          const double &g_magnitude, Eigen::Vector3d &trans_P_I,
                          Eigen::Vector3d &gravity_in_W);

  /**
   * @brief Obtain the tangent space basis of the gravity vector.
   *
   * @param[in] gravity Current gravity vector.
   * @return 3*2 Tangent space basis.
   */
  Eigen::Matrix<double, 3, 2> GravityTangentBasis(
      const Eigen::Vector3d &gravity);

  // Default initializer settings.
  double time_margin_ = 0.05;
  double element_interval_thresh_ = 0.1;
  double element_trans_thresh_ = 0.2;
  double element_rot_thresh_ = 5;
  int min_element_num_ = 30;
  int max_element_num_ = 300;
  int robust_kernel_coeff_ = 5;
  int refine_iter_num_ = 4;
};

}  // namespace xr_ucalib

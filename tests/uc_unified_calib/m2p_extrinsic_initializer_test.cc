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
#include <random>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include "xr_ucalib/uc_unified_calib/initializer/m2p_extrinsic_initializer.h"
// clang-format on

namespace xr_ucalib {
namespace {

/// @brief Test fixture for M2PExtrinsicInitializer tests.
class M2PExtrinsicInitializerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("%^[%l]%$ %v");
    initializer_ = M2PExtrinsicInitializer::Create();
  }

  /// @brief Generate synthetic pose and magnetometer data for testing.
  void GenerateSyntheticData(double duration, double dt_pose, double dt_mag,
                             double time_offset,
                             const Eigen::Quaterniond& q_P_M_gt,
                             const Eigen::Vector3d& mag_field_W,
                             double mag_noise_std = 0.0,
                             bool complex_motion = false) {
    pose_seq_ = PoseSequence::Create();
    mag_seq_ = MagSequence::Create();

    std::default_random_engine generator(42);
    std::normal_distribution<double> distribution(0.0, mag_noise_std);

    // Generate Pose Data
    for (double t = 0; t <= duration; t += dt_pose) {
      Eigen::Quaterniond q_W_P;

      if (complex_motion) {
        // Trajectory along circle, looking tangent
        Eigen::Vector3d tangent(-std::sin(0.5 * t), std::cos(0.5 * t), 0);
        Eigen::Vector3d z_axis(0, 0, 1);
        Eigen::Vector3d x_axis = tangent.normalized();
        Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
        z_axis = x_axis.cross(y_axis);
        Eigen::Matrix3d R_tgt;
        R_tgt.col(0) = x_axis;
        R_tgt.col(1) = y_axis;
        R_tgt.col(2) = z_axis;
        Eigen::Quaterniond q_base(R_tgt);

        // Add wobbles
        Eigen::Quaterniond q_wobble = Eigen::Quaterniond(
            Eigen::AngleAxisd(0.3 * std::sin(t), Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(0.3 * std::cos(2.0 * t),
                              Eigen::Vector3d::UnitY()));

        q_W_P = q_base * q_wobble;
        q_W_P.normalize();
      } else {
        // Simple rotation around X
        q_W_P = Eigen::Quaterniond(
            Eigen::AngleAxisd(t * 0.5, Eigen::Vector3d::UnitX()));
      }

      PoseFrame::Ptr pose = PoseFrame::Create();
      pose->timestamp = t;
      pose->rot_q = q_W_P;
      pose->trans = Eigen::Vector3d::Zero();
      pose_seq_->Add(pose);
    }

    // Generate Mag Data
    for (double t = 0; t <= duration; t += dt_mag) {
      double t_physical = t;
      double t_mag_stamp = t_physical - time_offset;

      Eigen::Quaterniond q_W_P;
      if (complex_motion) {
        Eigen::Vector3d tangent(-std::sin(0.5 * t_physical),
                                std::cos(0.5 * t_physical), 0);
        Eigen::Vector3d z_axis(0, 0, 1);
        Eigen::Vector3d x_axis = tangent.normalized();
        Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
        z_axis = x_axis.cross(y_axis);
        Eigen::Matrix3d R_tgt;
        R_tgt.col(0) = x_axis;
        R_tgt.col(1) = y_axis;
        R_tgt.col(2) = z_axis;
        Eigen::Quaterniond q_base(R_tgt);

        Eigen::Quaterniond q_wobble = Eigen::Quaterniond(
            Eigen::AngleAxisd(0.3 * std::sin(t_physical),
                              Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(0.3 * std::cos(2.0 * t_physical),
                              Eigen::Vector3d::UnitY()));

        q_W_P = q_base * q_wobble;
        q_W_P.normalize();
      } else {
        q_W_P = Eigen::Quaterniond(
            Eigen::AngleAxisd(t_physical * 0.5, Eigen::Vector3d::UnitX()));
      }

      // m_in_W = R_W_P * R_P_M * m_in_M
      // m_in_M = R_P_M^T * R_W_P^T * m_in_W
      Eigen::Vector3d m_M = q_P_M_gt.inverse() * q_W_P.inverse() * mag_field_W;

      // Add noise
      if (mag_noise_std > 0) {
        m_M.x() += distribution(generator);
        m_M.y() += distribution(generator);
        m_M.z() += distribution(generator);
      }

      // Enforce unit vector (simulating pre-calibrated sensor data)
      m_M.normalize();

      MagFrame::Ptr mag = MagFrame::Create();
      mag->timestamp = t_mag_stamp;
      mag->mag = m_M;
      mag_seq_->Add(mag);
    }
  }

  M2PExtrinsicInitializer::Ptr initializer_;
  PoseSequence::Ptr pose_seq_;
  MagSequence::Ptr mag_seq_;
};

/// @brief Test with perfect data, no noise.
TEST_F(M2PExtrinsicInitializerTest, PerfectData) {
  // Simple case
  Eigen::Quaterniond q_P_M_gt = Eigen::Quaterniond::Identity();
  Eigen::Vector3d mag_field_W_gt(0.0, 0.0, 1.0);
  double toff_P_M = 0.0;

  // Generate data: 30s. Enable complex motion.
  GenerateSyntheticData(30.0, 0.01, 0.005, toff_P_M, q_P_M_gt, mag_field_W_gt,
                        0.0, true);

  Eigen::Quaterniond q_P_M_est;
  Eigen::Vector3d mag_field_W_est;

  bool success = initializer_->EstimateFromSeq(pose_seq_, mag_seq_, toff_P_M,
                                               q_P_M_est, mag_field_W_est);
  EXPECT_TRUE(success);

  // Check rotation error
  Eigen::Quaterniond q_error = q_P_M_est.inverse() * q_P_M_gt;
  double angle_error =
      2.0 * std::atan2(q_error.vec().norm(), std::abs(q_error.w()));
  EXPECT_NEAR(angle_error, 0.0, 1e-4);

  // Check magnetic field error
  EXPECT_NEAR((mag_field_W_est - mag_field_W_gt).norm(), 0.0, 1e-4);
}

/// @brief Test with noisy data.
TEST_F(M2PExtrinsicInitializerTest, NoisyData) {
  Eigen::Quaterniond q_P_M_gt(
      Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitY()));
  Eigen::Vector3d mag_field_W_gt(0.0, 1.0, 0.0);
  double toff_P_M = -0.1;

  // With a bit of noise, complex rotation
  GenerateSyntheticData(60.0, 0.01, 0.01, toff_P_M, q_P_M_gt, mag_field_W_gt,
                        0.03, true);

  Eigen::Quaterniond q_P_M_est;
  Eigen::Vector3d mag_field_W_est;

  initializer_->set_mag_verification_thresh(
      0.2);  // Set loose threshold for somewhat noisy test

  bool success = initializer_->EstimateFromSeq(pose_seq_, mag_seq_, toff_P_M,
                                               q_P_M_est, mag_field_W_est);
  EXPECT_TRUE(success);

  Eigen::Quaterniond q_error = q_P_M_est.inverse() * q_P_M_gt;
  double angle_error =
      2.0 * std::atan2(q_error.vec().norm(), std::abs(q_error.w()));

  // Expect tolerance to be reasonable (< 3 degrees)
  EXPECT_LT(angle_error, 3.0 * M_PI / 180.0);

  // Expect mag field error to be reasonable (< 0.1)
  EXPECT_LT((mag_field_W_est - mag_field_W_gt).norm(), 0.1);
}

/// @brief Test with empty input sequences.
TEST_F(M2PExtrinsicInitializerTest, EmptyInput) {
  pose_seq_ = PoseSequence::Create();
  mag_seq_ = MagSequence::Create();
  Eigen::Quaterniond q_res;
  Eigen::Vector3d m_res;

  // Suppress error logs in test
  spdlog::set_level(spdlog::level::off);
  EXPECT_FALSE(
      initializer_->EstimateFromSeq(pose_seq_, mag_seq_, 0.0, q_res, m_res));
  spdlog::set_level(spdlog::level::warn);
}

}  // namespace
}  // namespace xr_ucalib

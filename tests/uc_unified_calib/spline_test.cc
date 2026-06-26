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

#include <ceres/jet.h>
#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include "xr_ucalib/uc_unified_calib/spline/ceres_spline_helper_jet.hpp"
#include "xr_ucalib/uc_unified_calib/spline/spline_bundle.hpp"

namespace xr_ucalib {
namespace {

using JetType = ceres::Jet<double, 6>;

/// @brief Test fixture for spline bundle tests.
class SplineBundleTest : public ::testing::Test {
 protected:
  void SetUp() override {
    spdlog::set_level(spdlog::level::warn);
    spdlog::set_pattern("%^[%l]%$ %v");

    std::vector<SplineInfo> spline_infos;
    // R3 Spline: 0.0 to 10.0s, dt=0.1.
    spline_infos.emplace_back("pos_spline", SplineType::EuclideanSpline, 0.0,
                              10.0, 0.1);
    // SO3 Spline: 0.0 to 10.0s, dt=0.1.
    spline_infos.emplace_back("rot_spline", SplineType::So3Spline, 0.0, 10.0,
                              0.1);

    spline_bundle_ = SplineBundle<5>::Create(spline_infos);
  }

  SplineBundle<5>::Ptr spline_bundle_;
};

/// @brief Test initialization and basic access of spline bundle.
TEST_F(SplineBundleTest, InitializationAndAccess) {
  ASSERT_NE(spline_bundle_, nullptr);

  // 1. Test R3d Spline Access.
  EXPECT_NO_THROW({ spline_bundle_->GetR3dSpline("pos_spline"); });
  auto& r3d_spline = spline_bundle_->GetR3dSpline("pos_spline");
  EXPECT_DOUBLE_EQ(r3d_spline.get_knot_interval(), 0.1);
  EXPECT_DOUBLE_EQ(r3d_spline.MinTime(), 0.0);
  EXPECT_GE(r3d_spline.MaxTime(), 10.0);

  // 2. Test So3d Spline Access.
  EXPECT_NO_THROW({ spline_bundle_->GetSo3dSpline("rot_spline"); });
  auto& so3d_spline = spline_bundle_->GetSo3dSpline("rot_spline");
  EXPECT_DOUBLE_EQ(so3d_spline.get_knot_interval(), 0.1);
  EXPECT_DOUBLE_EQ(so3d_spline.MinTime(), 0.0);
  EXPECT_GE(so3d_spline.MaxTime(), 10.0);

  // 3. Test Time Range Checks.
  EXPECT_TRUE(spline_bundle_->TimeInRangeForR3dSpline(5.0, "pos_spline"));
  EXPECT_TRUE(spline_bundle_->TimeInRangeForSo3dSpline(5.0, "rot_spline"));

  // Check boundary (start time).
  EXPECT_TRUE(spline_bundle_->TimeInRangeForR3dSpline(0.0, "pos_spline"));

  // Check out of bounds.
  EXPECT_FALSE(spline_bundle_->TimeInRangeForR3dSpline(-0.1, "pos_spline"));
  // Max time depends on how many knots were added during extension, it should
  // be at least > 10.0.
  EXPECT_FALSE(spline_bundle_->TimeInRangeForR3dSpline(
      so3d_spline.MaxTime() + 1.0, "pos_spline"));
}

/// @brief Test calculation of spline meta data.
TEST_F(SplineBundleTest, SplineMetaCalculation) {
  SplineBundle<5>::SplineMetaType r3d_meta;
  TimeSpanList valid_times = {{1.0, 2.0}, {5.5, 6.0}};

  // Test R3d Meta.
  EXPECT_TRUE(spline_bundle_->CalculateR3dSplineMeta("pos_spline", valid_times,
                                                     r3d_meta));
  EXPECT_FALSE(r3d_meta.segments.empty());

  // Verify basic segment properties.
  for (const auto& segment : r3d_meta.segments) {
    EXPECT_GT(segment.NumParameters(), 0);
    EXPECT_DOUBLE_EQ(segment.knot_interval, 0.1);
  }

  // Test So3 Meta.
  SplineBundle<5>::SplineMetaType so3_meta;
  EXPECT_TRUE(spline_bundle_->CalculateSo3dSplineMeta("rot_spline", valid_times,
                                                      so3_meta));
  EXPECT_FALSE(so3_meta.segments.empty());
}

/// @brief Test invalid access and error handling.
TEST_F(SplineBundleTest, InvalidAccess) {
  // Accessing non-existent spline should throw (map::at).
  EXPECT_ANY_THROW(spline_bundle_->GetR3dSpline("non_existent"));
  EXPECT_ANY_THROW(spline_bundle_->GetSo3dSpline("non_existent"));

  // Calculating meta for invalid times.
  SplineBundle<5>::SplineMetaType meta;
  TimeSpanList invalid_times = {{-1.0, -0.5}};
  spdlog::set_level(spdlog::level::off);
  EXPECT_FALSE(spline_bundle_->CalculateR3dSplineMeta("pos_spline",
                                                      invalid_times, meta));
  spdlog::set_level(spdlog::level::warn);
}

/// @brief Test evaluation of CeresSplineHelperJet for Euclidean splines.
TEST(CeresSplineHelperJetTest, EvaluateEuclideanConsistency) {
  // Setup knots and parameters.
  constexpr int Order = 4;
  constexpr int Dim = 1;

  double knots_storage[Order][Dim];
  double* knots_ptrs[Order];

  // Fill knots with some data
  for (int i = 0; i < Order; ++i) {
    knots_storage[i][0] = static_cast<double>(i * i);  // Simple quadratic
    knots_ptrs[i] = knots_storage[i];
  }

  double fraction = 0.5;
  double inv_dt = 10.0;  // dt = 0.1

  // 1. Evaluate with double
  Eigen::Matrix<double, Dim, 1> res_double;
  CeresSplineHelperJet<double, Order>::Evaluate<Dim, 0>(knots_ptrs, fraction,
                                                        inv_dt, &res_double);

  // 2. Evaluate with Jet
  // Convert knots to Jets
  JetType knots_jets_storage[Order][Dim];
  JetType* knots_jets_ptrs[Order];

  for (int i = 0; i < Order; ++i) {
    knots_jets_storage[i][0] = JetType(knots_storage[i][0]);
    knots_jets_ptrs[i] = knots_jets_storage[i];
  }

  JetType fraction_jet(fraction);
  Eigen::Matrix<JetType, Dim, 1> res_jet;

  CeresSplineHelperJet<JetType, Order>::Evaluate<Dim, 0>(
      knots_jets_ptrs, fraction_jet, inv_dt, &res_jet);

  // 3. Compare 'a' component of Jet with double result
  EXPECT_DOUBLE_EQ(res_double[0], res_jet[0].a);
}

/// @brief Test evaluation of CeresSplineHelperJet for SO(3) splines.
TEST(CeresSplineHelperJetTest, EvaluateLieSo3Consistency) {
  // Setup knots and parameters.
  constexpr int Order = 4;

  double knots_storage[Order][4];  // Quaternion xyzw
  double* knots_ptrs[Order];

  // Fill knots with some data
  for (int i = 0; i < Order; ++i) {
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    if (i > 0) {
      // Small rotation
      q = Eigen::AngleAxisd(0.1 * i, Eigen::Vector3d::UnitZ()) * q;
    }
    Eigen::Map<Eigen::Quaterniond> map_q(knots_storage[i]);
    map_q = q;
    knots_ptrs[i] = knots_storage[i];
  }

  double fraction = 0.3;
  double inv_dt = 10.0;

  // 1. Evaluate with double
  Sophus::SO3d res_so3_double;
  CeresSplineHelperJet<double, Order>::template EvaluateLie<Sophus::SO3>(
      knots_ptrs, fraction, inv_dt, &res_so3_double);

  // 2. Evaluate with Jet
  JetType knots_jets_storage[Order][4];
  JetType* knots_jets_ptrs[Order];

  for (int i = 0; i < Order; ++i) {
    for (int k = 0; k < 4; ++k) {
      knots_jets_storage[i][k] = JetType(knots_storage[i][k]);
    }
    knots_jets_ptrs[i] = knots_jets_storage[i];
  }

  JetType fraction_jet(fraction);
  Sophus::SO3<JetType> res_so3_jet;

  CeresSplineHelperJet<JetType, Order>::template EvaluateLie<Sophus::SO3>(
      knots_jets_ptrs, fraction_jet, inv_dt, &res_so3_jet);

  // 3. Compare
  // Compare quaternion components or rotation matrix
  EXPECT_NEAR(res_so3_double.unit_quaternion().x(),
              res_so3_jet.unit_quaternion().x().a, 1e-9);
  EXPECT_NEAR(res_so3_double.unit_quaternion().y(),
              res_so3_jet.unit_quaternion().y().a, 1e-9);
  EXPECT_NEAR(res_so3_double.unit_quaternion().z(),
              res_so3_jet.unit_quaternion().z().a, 1e-9);
  EXPECT_NEAR(res_so3_double.unit_quaternion().w(),
              res_so3_jet.unit_quaternion().w().a, 1e-9);
}

}  // namespace
}  // namespace xr_ucalib

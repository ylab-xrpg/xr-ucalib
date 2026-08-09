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

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "xr_ucalib/uc_cam_calib/cam_rig_calib/cam_reproj_cost.hpp"
#include "xr_ucalib/uc_common/calib_parameter/cam_radtan_thin_prism_fisheye_intrinsic.hpp"
#include "xr_ucalib/uc_common/calib_parameter/cam_unprojection.h"
#include "xr_ucalib/uc_unified_calib/calibrator/problem_builder.h"

namespace xr_ucalib {
namespace {

const std::vector<double> kFisheye624Parameters = {
    651.123, 655.123, 386.123, 511.123, -0.0232, 0.0924,  -0.0591, 0.003,
    0.0048,  -0.0009, 0.0002,  0.0005,  -0.0009, -0.0001, 0.00007, -0.00017};

// Keep this scalar reference implementation separate from the templated model
// so coefficient order and individual distortion terms are checked directly.
Eigen::Vector2d ReferenceProjection(const Eigen::Vector3d& ray,
                                    const std::vector<double>& parameters) {
  double u = ray.x() / ray.z();
  double v = ray.y() / ray.z();
  const double radius = std::sqrt(u * u + v * v);
  if (radius > std::numeric_limits<double>::epsilon()) {
    const double theta_div_radius = std::atan(radius) / radius;
    u *= theta_div_radius;
    v *= theta_div_radius;
  }

  const double theta2 = u * u + v * v;
  double radial = 1.0;
  double theta_power = 1.0;
  for (size_t i = 0; i < 6; ++i) {
    theta_power *= theta2;
    radial += parameters.at(4 + i) * theta_power;
  }

  const double x = radial * u;
  const double y = radial * v;
  const double x2 = x * x;
  const double y2 = y * y;
  const double xy = x * y;
  const double radius2 = x2 + y2;
  const double radius4 = radius2 * radius2;
  const double tangential_x =
      2.0 * parameters.at(11) * xy + parameters.at(10) * (radius2 + 2.0 * x2);
  const double tangential_y =
      2.0 * parameters.at(10) * xy + parameters.at(11) * (radius2 + 2.0 * y2);
  const double thin_prism_x =
      parameters.at(12) * radius2 + parameters.at(13) * radius4;
  const double thin_prism_y =
      parameters.at(14) * radius2 + parameters.at(15) * radius4;

  return {
      parameters.at(0) * (x + tangential_x + thin_prism_x) + parameters.at(2),
      parameters.at(1) * (y + tangential_y + thin_prism_y) + parameters.at(3)};
}

TEST(CamFisheyeIntrinsicTest, ProjectionAndUnprojectionMatchReference) {
  auto intrinsic = CamRadTanThinPrismFisheyeIntrinsic::Create();
  ASSERT_EQ(intrinsic->cam_model_type,
            CamModelType::RAD_TAN_THIN_PRISM_FISHEYE);
  ASSERT_EQ(intrinsic->parameter_size, 16);
  intrinsic->parameters = kFisheye624Parameters;

  const std::array<Eigen::Vector3d, 3> rays = {Eigen::Vector3d(0.0, 0.0, 1.0),
                                               Eigen::Vector3d(0.2, -0.3, 1.0),
                                               Eigen::Vector3d(1.0, 0.5, 1.0)};
  for (const Eigen::Vector3d& ray : rays) {
    Eigen::Vector2d projected;
    CamRadTanThinPrismFisheyeIntrinsic::Cam2Image(
        ray, intrinsic->parameters.data(), projected);
    const Eigen::Vector2d expected =
        ReferenceProjection(ray, intrinsic->parameters);
    EXPECT_NEAR(projected.x(), expected.x(), 1e-12);
    EXPECT_NEAR(projected.y(), expected.y(), 1e-12);

    Eigen::Vector2d normalized;
    ASSERT_TRUE(CamRadTanThinPrismFisheyeIntrinsic::Image2Cam(
        projected, intrinsic->parameters.data(), normalized));
    EXPECT_NEAR(normalized.x(), ray.x() / ray.z(), 1e-9);
    EXPECT_NEAR(normalized.y(), ray.y() / ray.z(), 1e-9);
  }
}

TEST(CamFisheyeIntrinsicTest, RestrictedModelAndInvalidInputAreHandled) {
  auto intrinsic = CamRadTanThinPrismFisheyeIntrinsic::Create(
      CamModelType::RAD_TAN_THIN_PRISM_FISHEYE_620);
  intrinsic->parameters.assign(16, 1.0);
  ASSERT_TRUE(ZeroFisheye624ConstantParams(intrinsic->cam_model_type,
                                           &intrinsic->parameters));
  for (const int index :
       GetFisheye624ConstantParams(intrinsic->cam_model_type)) {
    EXPECT_DOUBLE_EQ(intrinsic->parameters.at(index), 0.0);
  }

  std::vector<cv::Point2d> normalized;
  EXPECT_FALSE(UndistortCameraPoints(nullptr, {{1.0, 2.0}}, &normalized));
  intrinsic->parameters[0] = 0.0;
  ASSERT_TRUE(UndistortCameraPoints(intrinsic, {{1.0, 2.0}}, &normalized));
  ASSERT_EQ(normalized.size(), 1);
  EXPECT_FALSE(std::isfinite(normalized.front().x));
  EXPECT_FALSE(std::isfinite(normalized.front().y));
}

TEST(CamFisheyeIntrinsicTest, CeresUsesSixteenParameterIntrinsicBlock) {
  auto intrinsic = CamRadTanThinPrismFisheyeIntrinsic::Create();
  intrinsic->parameters = kFisheye624Parameters;
  const Eigen::Vector3d point_in_target(0.2, -0.3, 1.0);
  Eigen::Vector2d observation;
  CamRadTanThinPrismFisheyeIntrinsic::Cam2Image(
      point_in_target, intrinsic->parameters.data(), observation);

  std::unique_ptr<ceres::CostFunction> cost(
      CamReprojCost::Create(intrinsic, point_in_target, observation, 1.0));
  ASSERT_EQ(cost->parameter_block_sizes().size(), 7);
  ASSERT_EQ(cost->parameter_block_sizes().at(4), 16);

  std::array<double, 3> zero_translation = {0.0, 0.0, 0.0};
  std::array<double, 4> identity_rotation = {0.0, 0.0, 0.0, 1.0};
  const std::array<const double*, 7> parameter_blocks = {
      zero_translation.data(),      identity_rotation.data(),
      zero_translation.data(),      identity_rotation.data(),
      intrinsic->parameters.data(), zero_translation.data(),
      identity_rotation.data()};
  double residuals[2] = {1.0, 1.0};
  ASSERT_TRUE(cost->Evaluate(parameter_blocks.data(), residuals, nullptr));
  EXPECT_NEAR(residuals[0], 0.0, 1e-12);
  EXPECT_NEAR(residuals[1], 0.0, 1e-12);
}

TEST(CamFisheyeIntrinsicTest, Fisheye620ManifoldFixesThinPrismTerms) {
  auto system_config = SystemConfig::Create();
  system_config->unified_calib_config.fix_camera_intrinsics = false;
  auto calib_parameters = CalibParameters::Create();
  auto intrinsic = CamRadTanThinPrismFisheyeIntrinsic::Create(
      CamModelType::RAD_TAN_THIN_PRISM_FISHEYE_620);
  intrinsic->parameters.assign(16, 1.0);
  calib_parameters->cam_intrinsics["cam0"] = intrinsic;

  ProblemBuilder::Context context;
  context.system_config = system_config;
  context.calib_parameters = calib_parameters;
  auto builder = ProblemBuilder::Create(context);

  ceres::Problem::Options options;
  options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(options);
  CamConfig cam_config;
  cam_config.file_name = "cam0";
  cam_config.cam_model_type = CamModelType::RAD_TAN_THIN_PRISM_FISHEYE_620;

  ASSERT_TRUE(builder->ConfigureCameraIntrinsicParameterBlock(problem, "cam0",
                                                              cam_config));
  const ceres::Manifold* manifold =
      problem.GetManifold(intrinsic->parameters.data());
  ASSERT_NE(manifold, nullptr);
  EXPECT_EQ(manifold->AmbientSize(), 16);
  EXPECT_EQ(manifold->TangentSize(), 12);
  for (const int index :
       GetFisheye624ConstantParams(intrinsic->cam_model_type)) {
    EXPECT_DOUBLE_EQ(intrinsic->parameters.at(index), 0.0);
  }
}

}  // namespace
}  // namespace xr_ucalib

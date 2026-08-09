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
#include <functional>
#include <limits>
#include <string>

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include "xr_ucalib/uc_common/config/system_config.h"
// clang-format on

namespace xr_ucalib {
namespace {

SystemConfig::Ptr CreateValidCameraOnlyConfig() {
  auto config = SystemConfig::Create();
  CamConfig camera;
  camera.file_name = "cam0";
  camera.initial_focal_length = 240.0;
  camera.base_camera_flag = true;
  config->cam_configs.push_back(camera);

  TargetConfig target;
  target.target_idx = 0;
  config->target_configs.push_back(target);
  return config;
}

SystemConfig::Ptr CreateValidUnifiedConfig() {
  auto config = CreateValidCameraOnlyConfig();
  config->unified_calib_config.enable_unified_calib = true;

  ImuConfig imu;
  imu.file_name = "imu0.csv";
  imu.body_frame_flag = true;
  config->imu_configs.push_back(imu);

  MagConfig magnetometer;
  magnetometer.file_name = "mag0.csv";
  config->mag_configs.push_back(magnetometer);
  return config;
}

/// @brief Test fixture for system configuration tests.
class SystemConfigTest : public ::testing::Test {
 protected:
  void SetUp() override {
    spdlog::set_level(spdlog::level::warn);
    spdlog::set_pattern("%^[%l]%$ %v");
    base_dir_ = XR_UCALIB_TEST_DATA_DIR;
    output_path_ =
        std::string(XR_UCALIB_TEST_OUTPUT_DIR) + "/config_template.json";
    input_path_ = base_dir_ + "/input_config.json";
  }

  std::string base_dir_;
  std::string output_path_;
  std::string input_path_;
};

/// @brief Test generating a configuration template with multiple sensors.
TEST_F(SystemConfigTest, GenerateConfigTemplate) {
  constexpr size_t kCamNum = 4;
  constexpr size_t kImuNum = 2;
  constexpr size_t kMagNum = 1;
  constexpr size_t kFiducialNum = 3;

  auto system_config = xr_ucalib::SystemConfig::Create();

  for (size_t i = 0; i < kCamNum; ++i)
    system_config->cam_configs.emplace_back();
  for (size_t i = 0; i < kImuNum; ++i)
    system_config->imu_configs.emplace_back();
  for (size_t i = 0; i < kMagNum; ++i)
    system_config->mag_configs.emplace_back();
  for (size_t i = 0; i < kFiducialNum; ++i)
    system_config->target_configs.emplace_back();

  EXPECT_TRUE(system_config->ToJson(output_path_))
      << "Failed to generate config template";

  spdlog::info(
      "Generated config template with {} cameras, {} IMUs, {} magnetometers, "
      "and {} fiducials.",
      kCamNum, kImuNum, kMagNum, kFiducialNum);
}

/// @brief Test reading a system configuration from JSON.
TEST_F(SystemConfigTest, ReadSystemConfig) {
  auto system_config = xr_ucalib::SystemConfig::Create();
  EXPECT_TRUE(system_config->FromJson(input_path_))
      << "Failed to read system config from JSON";

  spdlog::info("Successfully read system config from JSON.");
}

/// @brief Reject numerical settings that would make calibration undefined.
TEST_F(SystemConfigTest, RejectsInvalidNumericSettings) {
  using ConfigMutation = std::function<void(const SystemConfig::Ptr&)>;
  const auto expect_invalid_camera_config = [](const ConfigMutation& mutate) {
    auto config = CreateValidCameraOnlyConfig();
    mutate(config);
    EXPECT_FALSE(config->CheckAndPrintConfig());
  };
  const auto expect_invalid_unified_config = [](const ConfigMutation& mutate) {
    auto config = CreateValidUnifiedConfig();
    mutate(config);
    EXPECT_FALSE(config->CheckAndPrintConfig());
  };

  expect_invalid_camera_config([](const auto& config) {
    config->cam_calib_config.cam_down_sample_rate = 0;
  });
  expect_invalid_camera_config([](const auto& config) {
    config->cam_calib_config.multi_thread_num = 0;
  });
  expect_invalid_camera_config([](const auto& config) {
    config->cam_calib_config.ceres_max_iterations = 0;
  });
  expect_invalid_camera_config([](const auto& config) {
    config->cam_configs[0].initial_focal_length = 0.0;
  });
  expect_invalid_camera_config(
      [](const auto& config) { config->cam_configs[0].noise = 0.0; });
  expect_invalid_camera_config([](const auto& config) {
    config->cam_configs[0].down_sample_rate_ucalib = 0;
  });
  expect_invalid_camera_config([](const auto& config) {
    config->target_configs[0].fiducial_size = 0.0;
  });
  expect_invalid_camera_config([](const auto& config) {
    config->target_configs[0].fiducial_spacing =
        std::numeric_limits<double>::quiet_NaN();
  });
  expect_invalid_camera_config(
      [](const auto& config) { config->target_configs[0].fiducial_rows = 0; });

  expect_invalid_unified_config([](const auto& config) {
    config->unified_calib_config.spline_knot_interval = 0.0;
  });
  expect_invalid_unified_config([](const auto& config) {
    config->unified_calib_config.gravity_magnitude = 0.0;
  });
  expect_invalid_unified_config([](const auto& config) {
    config->unified_calib_config.multi_thread_num = 0;
  });
  expect_invalid_unified_config([](const auto& config) {
    config->unified_calib_config.ceres_max_iterations = 0;
  });
  expect_invalid_unified_config(
      [](const auto& config) { config->imu_configs[0].frequency_hz = 0.0; });
  expect_invalid_unified_config(
      [](const auto& config) { config->imu_configs[0].noise[0] = 0.0; });
  expect_invalid_unified_config([](const auto& config) {
    config->imu_configs[0].down_sample_rate_ucalib = 0;
  });
  expect_invalid_unified_config(
      [](const auto& config) { config->mag_configs[0].noise = 0.0; });
  expect_invalid_unified_config([](const auto& config) {
    config->mag_configs[0].down_sample_rate_ucalib = 0;
  });
}

}  // namespace
}  // namespace xr_ucalib

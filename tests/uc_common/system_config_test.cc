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
#include <string>

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include "xr_ucalib/uc_common/config/system_config.h"
// clang-format on

namespace xr_ucalib {
namespace {

/// @brief Test fixture for system configuration tests.
class SystemConfigTest : public ::testing::Test {
 protected:
  void SetUp() override {
    spdlog::set_level(spdlog::level::warn);
    spdlog::set_pattern("%^[%l]%$ %v");
    base_dir_ = "../data/test_data_handheld";
    output_path_ = base_dir_ + "/config_template.json";
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

}  // namespace
}  // namespace xr_ucalib

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

#include "xr_ucalib/uc_common/calib_parameter/calib_parameters.h"
// clang-format on

namespace xr_ucalib {
namespace {

/// @brief Test fixture for calibration parameter tests.
class CalibParameterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    spdlog::set_level(spdlog::level::warn);
    spdlog::set_pattern("%^[%l]%$ %v");
    base_dir_ = "../data/test_data_handheld";
    config_path_ = base_dir_ + "/input_config.json";
    sensor_data_dir_ = base_dir_ + "/sensor_data";
    workspace_dir_ = base_dir_ + "/ucalib_ws";
    output_path_ = base_dir_ + "/output_calib_params.json";
  }

  std::string base_dir_;
  std::string config_path_;
  std::string sensor_data_dir_;
  std::string workspace_dir_;
  std::string output_path_;
};

/// @brief Test setting up default values and serialization
TEST_F(CalibParameterTest, DefaultValuesAndSerialization) {
  // 1. Load System Configuration.
  auto system_config = xr_ucalib::SystemConfig::Create();
  ASSERT_TRUE(system_config->FromJson(config_path_))
      << "Failed to read system config from " << config_path_;
  system_config->sensor_data_dir = sensor_data_dir_;
  system_config->workspace_dir = workspace_dir_;

  // 2. Load Sensor Data.
  auto sensor_manager = xr_ucalib::SensorManager::Create();
  ASSERT_TRUE(sensor_manager->LoadSensorData(system_config))
      << "Failed to load sensor data.";

  // 3. Create CalibParameters and Set Up Default Values.
  auto calib_params = xr_ucalib::CalibParameters::Create();
  ASSERT_TRUE(calib_params->SetUpDefaultValues(
      sensor_manager, system_config->unified_calib_config.enable_unified_calib))
      << "Failed to set up default calibration parameters.";

  // 4. Save to JSON.
  ASSERT_TRUE(calib_params->ToJson(output_path_))
      << "Failed to save calibration parameters to JSON.";

  // 5. Load from JSON.
  auto calib_params_loaded = xr_ucalib::CalibParameters::Create();
  ASSERT_TRUE(calib_params_loaded->FromJson(output_path_))
      << "Failed to load calibration parameters from JSON.";
}

}  // namespace
}  // namespace xr_ucalib

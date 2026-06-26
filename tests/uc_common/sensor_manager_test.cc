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
#include "xr_ucalib/uc_common/sensor_data/sensor_manager.h"
// clang-format on

namespace xr_ucalib {
namespace {

/// @brief Test fixture for sensor manager tests.
class SensorManagerTest : public ::testing::Test {
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

/// @brief Test loading sensor data and verifying its integrity.
TEST_F(SensorManagerTest, LoadSensorData) {
  // 1. Load System Configuration
  auto system_config = xr_ucalib::SystemConfig::Create();
  ASSERT_TRUE(system_config->FromJson(config_path_))
      << "Failed to read system config from " << config_path_;
  system_config->sensor_data_dir = sensor_data_dir_;
  system_config->workspace_dir = workspace_dir_;

  // 2. Create SensorManager and Load Data
  auto sensor_manager = xr_ucalib::SensorManager::Create();
  ASSERT_TRUE(sensor_manager->LoadSensorData(system_config))
      << "Failed to load sensor data.";

  // 3. Verify Camera Data
  for (const auto& config : system_config->cam_configs) {
    std::string label = config.file_name;
    auto seq = sensor_manager->GetCamSeqByLabel(label);
    EXPECT_NE(seq, nullptr) << "Camera sequence " << label << " is null.";
    if (seq) {
      EXPECT_FALSE(seq->Empty()) << "Camera sequence " << label << " is empty.";
    }

    double start_time = sensor_manager->GetCamStartTimeByLabel(label);
    double end_time = sensor_manager->GetCamEndTimeByLabel(label);

    EXPECT_GT(start_time, 0.0)
        << "Camera " << label << " start time should be positive.";
    EXPECT_GT(end_time, start_time)
        << "Camera " << label << " end time should be greater than start time.";
  }

  // 4. Verify IMU Data
  for (const auto& config : system_config->imu_configs) {
    std::string label = config.file_name;
    auto seq = sensor_manager->GetImuSeqByLabel(label);
    EXPECT_NE(seq, nullptr) << "IMU sequence " << label << " is null.";
    if (seq) {
      EXPECT_FALSE(seq->Empty()) << "IMU sequence " << label << " is empty.";
    }

    double start_time = sensor_manager->GetImuStartTimeByLabel(label);
    double end_time = sensor_manager->GetImuEndTimeByLabel(label);

    EXPECT_GT(start_time, 0.0)
        << "IMU " << label << " start time should be positive.";
    EXPECT_GT(end_time, start_time)
        << "IMU " << label << " end time should be greater than start time.";
  }

  // 5. Verify Magnetometer Data
  for (const auto& config : system_config->mag_configs) {
    std::string label = config.file_name;
    auto seq = sensor_manager->GetMagSeqByLabel(label);
    EXPECT_NE(seq, nullptr) << "Magnetometer sequence " << label << " is null.";
    if (seq) {
      EXPECT_FALSE(seq->Empty())
          << "Magnetometer sequence " << label << " is empty.";
    }

    double start_time = sensor_manager->GetMagStartTimeByLabel(label);
    double end_time = sensor_manager->GetMagEndTimeByLabel(label);

    EXPECT_GT(start_time, 0.0)
        << "Magnetometer " << label << " start time should be positive.";
    EXPECT_GT(end_time, start_time)
        << "Magnetometer " << label
        << " end time should be greater than start time.";
  }

  // 6. Verify Target Corners
  auto target_corners = sensor_manager->GetTargetCorners();
  EXPECT_NE(target_corners, nullptr) << "Target corners should not be null.";
  if (target_corners) {
    EXPECT_GT(target_corners->Size(), 0)
        << "Target corners should not be empty.";
  }
}

}  // namespace
}  // namespace xr_ucalib

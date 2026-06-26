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

#include <map>
#include <memory>
#include <string>

#include "xr_ucalib/uc_common/config/system_config.h"
#include "xr_ucalib/uc_common/sensor_data/sensor_data_types.h"
#include "xr_ucalib/uc_common/sensor_data/target_corner3d_generator.h"

namespace xr_ucalib {

/// @brief Sensor manager class for managing multiple sensors, including their
/// configurations and data.
class SensorManager {
 public:
  using Ptr = std::shared_ptr<SensorManager>;

  // Factory method to create a shared pointer instance.
  static SensorManager::Ptr Create() { return Ptr(new SensorManager()); }

  /**
   * @brief Load sensor data based on the provided system configuration.
   *
   * @param[in] system_config System configuration containing sensor settings.
   * @return true if loading is successful, false otherwise.
   */
  bool LoadSensorData(const SystemConfig::Ptr &system_config);

  // Getters for sensor configurations.
  const std::map<std::string, CamConfig> &GetAllCamConfigs() {
    return cam_configs_;
  }

  const std::map<std::string, ImuConfig> &GetAllImuConfigs() {
    return imu_configs_;
  }

  const std::map<std::string, MagConfig> &GetAllMagConfigs() {
    return mag_configs_;
  }

  const std::map<int, TargetConfig> &GetAllTargetConfigs() {
    return target_configs_;
  }

  // Getters for sensor data.
  const std::map<std::string, CamSequence::Ptr> &GetAllCamSequences() {
    return cam_sequences_;
  }

  const std::map<std::string, ImuSequence::Ptr> &GetAllImuSequences() {
    return imu_sequences_;
  }

  const std::map<std::string, MagSequence::Ptr> &GetAllMagSequences() {
    return mag_sequences_;
  }

  // Getters for sensor data sequences by label.
  CamSequence::Ptr GetCamSeqByLabel(const std::string &label);

  ImuSequence::Ptr GetImuSeqByLabel(const std::string &label);

  MagSequence::Ptr GetMagSeqByLabel(const std::string &label);

  // Getter for target corners.
  TargetCorner3D::Ptr GetTargetCorners() const { return target_corners_; }

  // Getters for start and end timestamps by label.
  double GetCamStartTimeByLabel(const std::string &label);

  double GetImuStartTimeByLabel(const std::string &label);

  double GetMagStartTimeByLabel(const std::string &label);

  double GetCamEndTimeByLabel(const std::string &label);

  double GetImuEndTimeByLabel(const std::string &label);

  double GetMagEndTimeByLabel(const std::string &label);

  /// @brief Get the common start time of all sensors (the latest start time).
  double GetCommonStartTime();

  /// @brief Get the common end time of all sensors (the earliest end time).
  double GetCommonEndTime();

 private:
  SensorManager() = default;

  // Containers for sensor configurations. key: label or target index; value:
  // configuration.
  std::map<std::string, CamConfig> cam_configs_;
  std::map<std::string, ImuConfig> imu_configs_;
  std::map<std::string, MagConfig> mag_configs_;
  std::map<int, TargetConfig> target_configs_;

  // Containers for sensor data sequences. key: label; value: data sequence.
  std::map<std::string, CamSequence::Ptr> cam_sequences_;
  std::map<std::string, ImuSequence::Ptr> imu_sequences_;
  std::map<std::string, MagSequence::Ptr> mag_sequences_;

  // Container for target corners.
  TargetCorner3D::Ptr target_corners_;
};

}  // namespace xr_ucalib

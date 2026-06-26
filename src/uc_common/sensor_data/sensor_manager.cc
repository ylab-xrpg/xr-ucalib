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
#include "xr_ucalib/uc_common/sensor_data/sensor_manager.h"

#include <filesystem>
#include <string>

#include <spdlog/spdlog.h>

#include "xr_ucalib/uc_common/sensor_data/cam_data_loader.h"
#include "xr_ucalib/uc_common/sensor_data/imu_data_loader.h"
#include "xr_ucalib/uc_common/sensor_data/mag_data_loader.h"
// clang-format on

namespace xr_ucalib {

bool SensorManager::LoadSensorData(const SystemConfig::Ptr &system_config) {
  spdlog::info("------------------- Load Sensor Data ------------------");

  if (system_config->sensor_data_dir == "") {
    spdlog::error(
        "Sensor data directory is not set in the system configuration.");
    return false;
  }

  if (system_config->workspace_dir == "") {
    spdlog::error(
        "Workspace directory is not set in the system configuration.");
    return false;
  }

  std::string work_dir = system_config->workspace_dir + "/sensor_manager";
  if (!std::filesystem::exists(work_dir)) {
    std::filesystem::create_directories(work_dir);
  }

  // Step 1: Load target configurations and generate 3D target corners.
  for (auto &target_config : system_config->target_configs) {
    int target_idx = target_config.target_idx;
    if (target_idx == -1) {
      spdlog::error("Target config found with invalid target_idx: {}",
                    target_config.target_idx);
      return false;
    }

    if (target_configs_.find(target_idx) == target_configs_.end()) {
      target_configs_[target_idx] = target_config;
    } else {
      spdlog::error("Duplicate target index: {}", target_idx);
      return false;
    }
  }

  TargetCorner3DGenerator::Ptr target_generator =
      TargetCorner3DGenerator::Create();

  if (!target_generator->Generate(target_configs_)) {
    spdlog::error("Fail to generate the target corners.");
    return false;
  }
  target_corners_ = target_generator->GetTargetCorners();

  spdlog::info("Generated {} 3D target corners.", target_corners_->Size());

  // ===========================================================================

  // Step 2: Load camera configurations and data.
  CamDataLoader::Ptr cam_loader = CamDataLoader::Create();
  FiducialType fiducial_type = target_configs_.begin()->second.fiducial_type;
  cam_loader->SetFiducialType(fiducial_type);
  cam_loader->SetNumThreads(system_config->cam_calib_config.multi_thread_num);

  for (auto &cam_config : system_config->cam_configs) {
    std::string file_name = cam_config.file_name;
    if (file_name == "") {
      spdlog::error("Camera config found with empty file_name.");
      return false;
    }

    // If detections file exists, read from it. Otherwise, load raw data and
    // perform detection.
    std::string detection_path =
        work_dir + "/" + file_name + "_detections.json";
    if (std::filesystem::exists(detection_path) &&
        std::filesystem::is_regular_file(detection_path)) {
      spdlog::info(
          "Detections file exists for camera: {}, reading from existing file.",
          file_name);
      if (!cam_loader->ReadDetections(detection_path)) {
        spdlog::error("Failed to read detections from existing file: {}",
                      detection_path);
        return false;
      }
    } else {
      std::string cam_data_path =
          system_config->sensor_data_dir + "/" + file_name;
      if (!cam_loader->Load(cam_data_path)) {
        spdlog::error("Failed to load camera data from: {}", cam_data_path);
        return false;
      }

      cam_loader->SaveDetections(work_dir + "/" + file_name +
                                 "_detections.json");
    }

    cam_config.width = cam_loader->GetWidth();
    cam_config.height = cam_loader->GetHeight();
    if (cam_configs_.find(file_name) == cam_configs_.end()) {
      cam_configs_[file_name] = cam_config;
      cam_sequences_[file_name] = cam_loader->GetSequence();
    } else {
      spdlog::error("Duplicate camera file name: {}", file_name);
      return false;
    }

    spdlog::info("Loaded {} camera frames from {}.",
                 cam_sequences_[file_name]->Size(), file_name);

    // Show detections if required.
    if (cam_config.detection_display_mode != DetectionDisplayMode::NONE) {
      bool step_mode =
          (cam_config.detection_display_mode == DetectionDisplayMode::STEP);
      cam_loader->ShowDetections(step_mode);
    }
  }

  // ===========================================================================

  // Step 3: Validate that all detected keypoint IDs exist in the generated
  // target corners. Remove keypoints with undefined IDs.
  for (const auto &cam_pair : cam_sequences_) {
    const std::string &cam_label = cam_pair.first;
    const CamSequence::Ptr &seq = cam_pair.second;

    for (const auto &frame : seq->GetFrames()) {
      if (!frame) {
        spdlog::error("Null frame found in camera sequence: {}", cam_label);
        return false;
      }

      // Remove keypoints that are not in target corners.
      for (auto it = frame->keypoints.begin(); it != frame->keypoints.end();) {
        int id = it->first;
        if (!target_corners_->Contains(id)) {
          spdlog::warn(
              "Detected keypoint ID {} in camera '{}' (timestamp: {:.6f}) is "
              "not defined in the target configuration. Removing it.",
              id, cam_label, frame->timestamp);
          it = frame->keypoints.erase(it);
        } else {
          ++it;
        }
      }
    }
  }

  // ===========================================================================

  // Step 4: Load IMU configurations and data.
  ImuDataLoader::Ptr imu_loader = ImuDataLoader::Create();

  for (const auto &imu_config : system_config->imu_configs) {
    std::string file_name = imu_config.file_name;
    if (file_name == "") {
      spdlog::error("Imu config found with empty file_name.");
      return false;
    }

    std::string imu_data_path =
        system_config->sensor_data_dir + "/" + file_name;
    if (!imu_loader->Load(imu_data_path)) {
      spdlog::error("Failed to load IMU data from: {}", imu_data_path);
      return false;
    }

    if (imu_configs_.find(file_name) == imu_configs_.end()) {
      imu_configs_[file_name] = imu_config;
      imu_sequences_[file_name] = imu_loader->GetSequence();
    } else {
      spdlog::error("Duplicate IMU file name: {}", file_name);
      return false;
    }

    spdlog::info("Loaded {} IMU frames from {}.",
                 imu_sequences_[file_name]->Size(), file_name);
  }

  // ===========================================================================

  // Step 5: Load magnetometer configurations and data.
  MagDataLoader::Ptr mag_loader = MagDataLoader::Create();

  for (const auto &mag_config : system_config->mag_configs) {
    std::string file_name = mag_config.file_name;
    if (file_name == "") {
      spdlog::error("Magnetometer config found with empty file_name.");
      return false;
    }

    std::string mag_data_path =
        system_config->sensor_data_dir + "/" + file_name;
    if (!mag_loader->Load(mag_data_path)) {
      spdlog::error("Failed to load magnetometer data from: {}", mag_data_path);
      return false;
    }

    if (mag_configs_.find(file_name) == mag_configs_.end()) {
      mag_configs_[file_name] = mag_config;
      mag_sequences_[file_name] = mag_loader->GetSequence();
    } else {
      spdlog::error("Duplicate magnetometer file name: {}", file_name);
      return false;
    }

    spdlog::info("Loaded {} magnetometer frames from {}.",
                 mag_sequences_[file_name]->Size(), file_name);
  }

  // ===========================================================================

  return true;
}

// ===========================================================================
// ==================== Getters for sensor configurations ====================
// ===========================================================================
CamSequence::Ptr SensorManager::GetCamSeqByLabel(const std::string &label) {
  if (cam_sequences_.find(label) != cam_sequences_.end()) {
    return cam_sequences_.at(label);
  } else {
    spdlog::error("Camera sequence not found for label: {}", label);
    return nullptr;
  }
}

ImuSequence::Ptr SensorManager::GetImuSeqByLabel(const std::string &label) {
  if (imu_sequences_.find(label) != imu_sequences_.end()) {
    return imu_sequences_.at(label);
  } else {
    spdlog::error("IMU sequence not found for label: {}", label);
    return nullptr;
  }
}

MagSequence::Ptr SensorManager::GetMagSeqByLabel(const std::string &label) {
  if (mag_sequences_.find(label) != mag_sequences_.end()) {
    return mag_sequences_.at(label);
  } else {
    spdlog::error("Magnetometer sequence not found for label: {}", label);
    return nullptr;
  }
}

// ===========================================================================
// ==================== Getters for start / end timestamps ===================
// ===========================================================================
double SensorManager::GetCamStartTimeByLabel(const std::string &label) {
  auto cam_seq = GetCamSeqByLabel(label);
  if (cam_seq && !cam_seq->Empty()) {
    return cam_seq->Front()->timestamp;
  } else {
    spdlog::error("Camera sequence is empty for label: {}", label);
    return -1.0;
  }
}

double SensorManager::GetImuStartTimeByLabel(const std::string &label) {
  auto imu_seq = GetImuSeqByLabel(label);
  if (imu_seq && !imu_seq->Empty()) {
    return imu_seq->Front()->timestamp;
  } else {
    spdlog::error("IMU sequence is empty for label: {}", label);
    return -1.0;
  }
}

double SensorManager::GetMagStartTimeByLabel(const std::string &label) {
  auto mag_seq = GetMagSeqByLabel(label);
  if (mag_seq && !mag_seq->Empty()) {
    return mag_seq->Front()->timestamp;
  } else {
    spdlog::error("Magnetometer sequence is empty for label: {}", label);
    return -1.0;
  }
}

double SensorManager::GetCamEndTimeByLabel(const std::string &label) {
  auto cam_seq = GetCamSeqByLabel(label);
  if (cam_seq && !cam_seq->Empty()) {
    return cam_seq->Back()->timestamp;
  } else {
    spdlog::error("Camera sequence is empty for label: {}", label);
    return -1.0;
  }
}

double SensorManager::GetImuEndTimeByLabel(const std::string &label) {
  auto imu_seq = GetImuSeqByLabel(label);
  if (imu_seq && !imu_seq->Empty()) {
    return imu_seq->Back()->timestamp;
  } else {
    spdlog::error("IMU sequence is empty for label: {}", label);
    return -1.0;
  }
}

double SensorManager::GetMagEndTimeByLabel(const std::string &label) {
  auto mag_seq = GetMagSeqByLabel(label);
  if (mag_seq && !mag_seq->Empty()) {
    return mag_seq->Back()->timestamp;
  } else {
    spdlog::error("Magnetometer sequence is empty for label: {}", label);
    return -1.0;
  }
}

// ===========================================================================
// ====================== Getters for common timestamps ======================
// ===========================================================================
double SensorManager::GetCommonStartTime() {
  double start_time = -1.0;
  bool first = true;

  auto update_start = [&](double t) {
    if (first) {
      start_time = t;
      first = false;
    } else {
      if (t > start_time) {
        start_time = t;
      }
    }
  };

  for (const auto &pair : cam_sequences_) {
    if (pair.second && !pair.second->Empty()) {
      update_start(pair.second->Front()->timestamp);
    }
  }
  for (const auto &pair : imu_sequences_) {
    if (pair.second && !pair.second->Empty()) {
      update_start(pair.second->Front()->timestamp);
    }
  }
  for (const auto &pair : mag_sequences_) {
    if (pair.second && !pair.second->Empty()) {
      update_start(pair.second->Front()->timestamp);
    }
  }

  return start_time;
}

double SensorManager::GetCommonEndTime() {
  double end_time = -1.0;
  bool first = true;

  auto update_end = [&](double t) {
    if (first) {
      end_time = t;
      first = false;
    } else {
      if (t < end_time) {
        end_time = t;
      }
    }
  };

  for (const auto &pair : cam_sequences_) {
    if (pair.second && !pair.second->Empty()) {
      update_end(pair.second->Back()->timestamp);
    }
  }
  for (const auto &pair : imu_sequences_) {
    if (pair.second && !pair.second->Empty()) {
      update_end(pair.second->Back()->timestamp);
    }
  }
  for (const auto &pair : mag_sequences_) {
    if (pair.second && !pair.second->Empty()) {
      update_end(pair.second->Back()->timestamp);
    }
  }

  return end_time;
}

}  // namespace xr_ucalib

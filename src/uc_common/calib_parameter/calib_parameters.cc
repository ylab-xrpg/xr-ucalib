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
#include "xr_ucalib/uc_common/calib_parameter/calib_parameters.h"

#include <fstream>

#include <spdlog/spdlog.h>
// clang-format on

namespace xr_ucalib {

bool CalibParameters::SetUpDefaultValues(
    const SensorManager::Ptr& sensor_manager, bool enable_unified_calib) {
  spdlog::info("Set up default calibration parameters...");

  // Step 1: Set up camera parameters.
  for (const auto& [label, cam_config] : sensor_manager->GetAllCamConfigs()) {
    // Set up camera spatiotemporal extrinsics.
    if (cam_config.base_camera_flag) {
      base_camera_label = label;
    }
    trans_Cb_Ci[label] = Sophus::Vector3d::Zero();
    rot_Cb_Ci[label] = Sophus::SO3d();
    time_offset_Cb_Ci[label] = 0.0;

    // Set up camera intrinsic.
    auto cam_intrinsic = CamIntrinsicBase::Ptr();
    if (cam_config.cam_model_type == CamModelType::RADTAN) {
      cam_intrinsic = CamRadtanIntrinsic::Create();
    } else if (cam_config.cam_model_type == CamModelType::EQUIDISTANT) {
      cam_intrinsic = CamEqdistIntrinsic::Create();
    } else {
      spdlog::error("Unsupported camera model type: {}",
                    CamModelTypeToString(cam_config.cam_model_type));
      return false;
    }
    cam_intrinsic->initial_focal_length = cam_config.initial_focal_length;
    if (cam_config.width <= 0 || cam_config.height <= 0) {
      spdlog::error(
          "Invalid image dimensions for camera {}: {}x{}, please read ", label,
          cam_config.width, cam_config.height);
      return false;
    }
    cam_intrinsic->width = cam_config.width;
    cam_intrinsic->height = cam_config.height;
    cam_intrinsics[label] = cam_intrinsic;
  }

  // ===========================================================================

  // Step 2: Set up target parameters.
  bool first_target = true;
  for (const auto& [index, target_config] :
       sensor_manager->GetAllTargetConfigs()) {
    if (first_target) {
      first_target = false;
      world_target_index = index;
    }

    trans_W_Ti[index] = Sophus::Vector3d::Zero();
    rot_W_Ti[index] = Sophus::SO3d();
  }

  if (world_target_index == -1 || base_camera_label.empty()) {
    spdlog::error(
        "World target index or base camera label not set properly. "
        "World target index: {}, base camera label: {}",
        world_target_index, base_camera_label);
    return false;
  }

  if (!enable_unified_calib) {
    param_status = ParamStatus::SETUP;
    return true;
  }

  // ===========================================================================

  // Step 3: Set up IMU parameters.
  for (const auto& [label, imu_config] : sensor_manager->GetAllImuConfigs()) {
    // Set up IMU spatiotemporal extrinsics.
    if (imu_config.body_frame_flag) {
      body_frame_label = label;
    }
    trans_B_Ii[label] = Sophus::Vector3d::Zero();
    rot_B_Ii[label] = Sophus::SO3d();
    time_offset_B_Ii[label] = 0.0;

    // Set up IMU intrinsic.
    auto imu_intrinsic = ImuIntrinsic::Create();
    imu_intrinsics[label] = imu_intrinsic;
  }

  // Step up gravity-aligned frame rotation.
  gravity_in_W = Eigen::Vector3d::UnitZ();

  // ===========================================================================

  // Step 4: Set up magnetometer parameters.
  for (const auto& [label, mag_config] : sensor_manager->GetAllMagConfigs()) {
    // Set up magnetometer spatiotemporal extrinsics.
    rot_B_Mi[label] = Sophus::SO3d();
    time_offset_B_Mi[label] = 0.0;
  }

  mag_in_W = Eigen::Vector3d::UnitX();

  // ===========================================================================

  if (body_frame_label.empty()) {
    spdlog::error("Body frame label not set properly: {}", body_frame_label);
    return false;
  }

  param_status = ParamStatus::SETUP;

  return true;
}

bool CalibParameters::FromJson(const std::string& input_path) {
  spdlog::info("Read calibration parameters from: {}", input_path);

  try {
    std::ifstream input_file(input_path);
    if (!input_file.is_open()) {
      spdlog::error("Failed to open input JSON file: {}", input_path);
      return false;
    }

    nlohmann::json nlm_json;
    input_file >> nlm_json;
    input_file.close();

    // Step 1: Deserialize basic fields using the macro.
    nlm_json.get_to(*this);

    if (nlm_json.contains("param_status")) {
      param_status = nlm_json["param_status"].get<uint8_t>();
    }

    // Step 2: Manually deserialize Camera Intrinsics (Polymorphic).
    cam_intrinsics.clear();
    if (nlm_json.contains("cam_intrinsics")) {
      for (const auto& [label, j_cam] : nlm_json["cam_intrinsics"].items()) {
        CamModelType type = j_cam.at("cam_model_type").get<CamModelType>();
        CamIntrinsicBase::Ptr cam_ptr;

        if (type == CamModelType::RADTAN) {
          cam_ptr = CamRadtanIntrinsic::Create();
        } else if (type == CamModelType::EQUIDISTANT) {
          cam_ptr = CamEqdistIntrinsic::Create();
        } else {
          spdlog::error("Unknown camera model type for {}: {}", label,
                        CamModelTypeToString(type));
          continue;
        }

        cam_ptr->width = j_cam.at("width").get<int>();
        cam_ptr->height = j_cam.at("height").get<int>();
        cam_ptr->parameters = j_cam.at("parameters").get<std::vector<double>>();

        // Validate parameter size
        if (cam_ptr->parameters.size() !=
            static_cast<size_t>(cam_ptr->parameter_size)) {
          spdlog::warn("Parameter size mismatch for {}. Expected {}, got {}",
                       label, cam_ptr->parameter_size,
                       cam_ptr->parameters.size());
        }

        cam_intrinsics[label] = cam_ptr;
      }
    }

    // Step 3: Manually deserialize IMU Intrinsics.
    imu_intrinsics.clear();
    if (nlm_json.contains("imu_intrinsics")) {
      for (const auto& [label, j_imu] : nlm_json["imu_intrinsics"].items()) {
        auto imu_ptr = ImuIntrinsic::Create();
        j_imu.get_to(*imu_ptr);
        imu_intrinsics[label] = imu_ptr;
      }
    }

    // Step 4: Manually deserialize Magnetometer Intrinsics.
    // mag_intrinsics.clear();
    // if (nlm_json.contains("mag_intrinsics")) {
    //   for (const auto& [label, j_mag] : nlm_json["mag_intrinsics"].items()) {
    //     auto mag_ptr = MagIntrinsic::Create();
    //     j_mag.get_to(*mag_ptr);
    //     mag_intrinsics[label] = mag_ptr;
    //   }
    // }

  } catch (const std::exception& e) {
    spdlog::error("JSON parsing error: {}", e.what());
    return false;
  } catch (...) {
    spdlog::error("Unknown error reading JSON.");
    return false;
  }

  return true;
}

bool CalibParameters::ToJson(const std::string& output_path) {
  spdlog::info("Write calibration parameters to: {}", output_path);

  try {
    std::ofstream output_file(output_path);
    if (!output_file.is_open()) {
      spdlog::error("Failed to open output JSON: {}", output_path);
      return false;
    }

    nlohmann::json nlm_json;

    // Step 1: Serialize basic fields using the macro.
    nlm_json = *this;

    nlm_json["param_status"] = param_status;

    // Step 2: Manually serialize Camera Intrinsics.
    nlm_json["cam_intrinsics"] = nlohmann::json::object();
    for (const auto& [label, cam_ptr] : cam_intrinsics) {
      nlohmann::json j_cam;
      j_cam["cam_model_type"] = cam_ptr->cam_model_type;
      j_cam["width"] = cam_ptr->width;
      j_cam["height"] = cam_ptr->height;
      j_cam["parameters"] = cam_ptr->parameters;
      nlm_json["cam_intrinsics"][label] = j_cam;
    }

    // Step 3: Manually serialize IMU Intrinsics.
    nlm_json["imu_intrinsics"] = nlohmann::json::object();
    for (const auto& [label, imu_ptr] : imu_intrinsics) {
      nlm_json["imu_intrinsics"][label] = *imu_ptr;
    }

    // Step 4: Manually serialize Magnetometer Intrinsics.
    // nlm_json["mag_intrinsics"] = nlohmann::json::object();
    // for (const auto& [label, mag_ptr] : mag_intrinsics) {
    //   nlm_json["mag_intrinsics"][label] = *mag_ptr;
    // }

    // Serialize.
    output_file << nlm_json.dump(2);

  } catch (const std::exception& e) {
    spdlog::error("JSON writing error: {}", e.what());
    return false;
  } catch (...) {
    spdlog::error("Unknown error writing JSON.");
    return false;
  }

  return true;
}

}  // namespace xr_ucalib
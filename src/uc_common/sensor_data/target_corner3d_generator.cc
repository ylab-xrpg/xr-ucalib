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
#include "xr_ucalib/uc_common/sensor_data/target_corner3d_generator.h"

#include <fstream>

#include <spdlog/spdlog.h>
// clang-format on

namespace xr_ucalib {

bool TargetCorner3DGenerator::Generate(
    const std::map<int, TargetConfig>& target_configs) {
  if (target_configs.empty()) {
    spdlog::error("No target configurations provided for corner generation.");
    return false;
  }

  // Create a new TargetCorner3D container.
  target_corners_ = TargetCorner3D::Create();

  // Process targets in order by their keys (which are target_idx values).
  for (const auto& [target_idx, config] : target_configs) {
    int fiducial_rows = config.fiducial_rows;
    int fiducial_cols = config.fiducial_cols;
    double fiducial_size = config.fiducial_size;
    double fiducial_spacing = config.fiducial_spacing;

    // Corner ID starts from the configured start_id.
    int corner_id = config.start_id;
    int min_id = corner_id;

    // Process fiducials in row-major order
    for (int row = 0; row < fiducial_rows; ++row) {
      for (int col = 0; col < fiducial_cols; ++col) {
        // Base coordinates for current fiducial.
        double base_x = col * (1 + fiducial_spacing) * fiducial_size;

        double base_y = row * (1 + fiducial_spacing) * fiducial_size;

        // For each fiducial, generate 4 corners(in order 0, 1, 2, 3).
        // Corner 0: top-left (0, size)
        // Corner 1: top-right (size, size)
        // Corner 2: bottom-right (size, 0)
        // Corner 3: bottom-left (0, 0)
        double corner_offsets_x[4] = {0.0, fiducial_size, fiducial_size, 0.0};
        double corner_offsets_y[4] = {fiducial_size, fiducial_size, 0.0, 0.0};
        for (int corner_idx = 0; corner_idx < 4; ++corner_idx) {
          Corner3D corner;
          corner.target_idx = target_idx;

          // Calculate local 3D coordinates
          double x = base_x + corner_offsets_x[corner_idx];
          double y = base_y + corner_offsets_y[corner_idx];
          double z = 0.0;

          // Only local position is set here. Global position will be derived
          // from the relative pose between the multiple targets.
          corner.position_local = Eigen::Vector3d(x, y, z);

          // Add corner to the container with unique ID.
          if (target_corners_->Contains(corner_id)) {
            spdlog::error(
                "Duplicate corner ID {} detected. Please check target "
                "configurations for overlapping ID ranges.",
                corner_id);
            return false;
          }
          target_corners_->Add(corner_id, corner);
          // Increment unique corner ID.
          ++corner_id;
        }
      }
    }

    int max_id = corner_id - 1;
    target_corners_->target_id_ranges[target_idx] = {min_id, max_id};
  }

  return true;
}

bool TargetCorner3DGenerator::SaveTargetCorners(const std::string& file_path) {
  if (!target_corners_ || target_corners_->Empty()) {
    spdlog::warn("No target corners to save. Please generate data first.");
    return false;
  }

  spdlog::info("Saving target corners to: {}", file_path);

  try {
    std::ofstream output_file(file_path);
    if (!output_file.is_open()) {
      spdlog::error("Failed to open output JSON file: {}", file_path);
      return false;
    }

    nlohmann::json nlm_json = *target_corners_;

    // Serialize.
    output_file << nlm_json.dump(2);
    output_file.close();

    spdlog::info("Successfully saved {} target corners to JSON.",
                 target_corners_->Size());

  } catch (const std::exception& e) {
    spdlog::error("JSON writing error: {}", e.what());
    return false;
  } catch (...) {
    spdlog::error("Unknown error writing JSON.");
    return false;
  }

  return true;
}

bool TargetCorner3DGenerator::ReadTargetCorners(const std::string& file_path) {
  spdlog::info("Reading target corners from: {}", file_path);

  try {
    std::ifstream input_file(file_path);
    if (!input_file.is_open()) {
      spdlog::error("Failed to open input JSON file: {}", file_path);
      return false;
    }

    nlohmann::json nlm_json;
    input_file >> nlm_json;
    input_file.close();

    // Deserialize.
    if (!target_corners_) {
      target_corners_ = TargetCorner3D::Create();
    }
    nlm_json.get_to(*target_corners_);

    spdlog::info("Successfully loaded {} target corners from JSON.",
                 target_corners_->Size());

  } catch (const std::exception& e) {
    spdlog::error("JSON parsing error: {}", e.what());
    return false;
  } catch (...) {
    spdlog::error("Unknown error reading JSON.");
    return false;
  }

  return true;
}

}  // namespace xr_ucalib
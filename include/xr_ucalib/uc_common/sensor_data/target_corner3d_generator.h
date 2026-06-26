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

// clang-format off
#include <map>
#include <memory>

#include <Eigen/Eigen>
#include <nlohmann/json.hpp>

#include "xr_ucalib/uc_common/config/target_config.h"
// clang-format on

namespace xr_ucalib {

/// @brief Single 3D corner point of a calibration target.
struct Corner3D {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Index of the calibration target this corner belongs to.
  int target_idx = -1;

  // 3D position in the local coordinate frame of the current calibration
  // target.
  Eigen::Vector3d position_local = Eigen::Vector3d::Zero();

  // 3D position in the global coordinate frame. Used for multi-target.
  Eigen::Vector3d position_global = Eigen::Vector3d::Zero();
};

// Nlohmann JSON serialization for Corner3D.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Corner3D, target_idx,
                                                position_local,
                                                position_global);

/// @brief Container for multiple 3D corners of calibration targets.
struct TargetCorner3D {
 public:
  using Ptr = std::shared_ptr<TargetCorner3D>;

  // Factory method to create a shared pointer instance.
  static Ptr Create() { return Ptr(new TargetCorner3D()); }

  /**
   * Container for multiple 3D corners of the target boards.
   * Key: Unique ID of the corner, Value: Corner3D.
   * Note that the structure of our corners in a single target is as follows:
   *
   *         8----------9   12--------13
   *         |          |   |          |
   *         |Fiducial 3|   |Fiducial 4|
   *         |          |   |          |
   *         11--------10   15--------14
   *
   *         0----------1   4----------5
   *         |          |   |          |
   *         |Fiducial 1|   |Fiducial 2|
   *   y     |          |   |          |
   *   ^     3----------2   7----------6
   *   |-->x
   *
   * For multiple targets, the corner IDs are also unique and managed in a
   * single map.
   */
  std::map<int, Corner3D> corners;

  // Container for the ID ranges of corners for each target. This is useful for
  // quickly determining which target a corner belongs to based on its ID.
  // Key: target_idx, Value: {min_id, max_id} (inclusive)
  std::map<int, std::pair<int, int>> target_id_ranges;

  // STL-like interface methods.
  void Add(int id, const Corner3D& corner) { corners[id] = corner; }
  void Add(int id, Corner3D&& corner) { corners[id] = std::move(corner); }

  const Corner3D& operator[](int id) const { return corners.at(id); }
  Corner3D& operator[](int id) { return corners[id]; }

  const Corner3D& At(int id) const { return corners.at(id); }
  Corner3D& At(int id) { return corners.at(id); }

  bool Contains(int id) const { return corners.find(id) != corners.end(); }

  auto begin() { return corners.begin(); }
  auto end() { return corners.end(); }
  auto begin() const { return corners.begin(); }
  auto end() const { return corners.end(); }
  auto cbegin() const { return corners.cbegin(); }
  auto cend() const { return corners.cend(); }

  size_t Size() const { return corners.size(); }
  bool Empty() const { return corners.empty(); }

  void Clear() {
    corners.clear();
    target_id_ranges.clear();
  }

  /// @brief Get the target index for a given corner ID based on the target ID
  /// ranges.
  int GetTargetIdx(int corner_id) const {
    for (const auto& [idx, range] : target_id_ranges) {
      if (corner_id >= range.first && corner_id <= range.second) return idx;
    }
    return -1;
  }

 private:
  TargetCorner3D() = default;
};

// Nlohmann JSON serialization for TargetCorner3D.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(TargetCorner3D, corners,
                                                target_id_ranges);

/// @brief Generator for 3D corner points of calibration targets.
class TargetCorner3DGenerator {
 public:
  using Ptr = std::shared_ptr<TargetCorner3DGenerator>;

  // Factory method to create a shared pointer instance.
  static Ptr Create() { return Ptr(new TargetCorner3DGenerator()); }

  /**
   * @brief Generate 3D corner points for the given target configurations.
   * This function support multiple targets, the corner IDs are incremental and
   * unique.
   *
   * @param target_configs Target configurations mapped by their target indices.
   * @return true if generation is successful, false otherwise.
   */
  bool Generate(const std::map<int, TargetConfig>& target_configs);

  // Get the generated target corners.
  TargetCorner3D::Ptr GetTargetCorners() const { return target_corners_; }

  // Save the generated target corners to a JSON file (Serialize).
  bool SaveTargetCorners(const std::string& file_path);

  // Read the target corners from a JSON file (Deserialize).
  bool ReadTargetCorners(const std::string& file_path);

 private:
  TargetCorner3DGenerator() = default;

  // 3D corners of the calibration targets.
  TargetCorner3D::Ptr target_corners_;
};

}  // namespace xr_ucalib

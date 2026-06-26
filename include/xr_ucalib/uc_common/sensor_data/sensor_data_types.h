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
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <nlohmann/json.hpp>

#include "xr_ucalib/uc_common/config/json_adapter.hpp"
// clang-format on

namespace xr_ucalib {

// ===========================================================================
// ============================ Sensor Frame types ===========================
// ===========================================================================

/// @brief Structure for a single camera frame.
struct CamFrame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CamFrame>;

  // Factory method to create a shared pointer instance.
  static Ptr Create() { return Ptr(new CamFrame()); }

  // Timestamp in seconds.
  double timestamp = -1.;

  // Full image path.
  std::string img_path;

  // 2D keypoints detected in the image. Key: landmark ID, Value: (x, y).
  std::map<int, Eigen::Vector2d> keypoints;

  // Camera pose (Transform form camera to target).
  Eigen::Vector3d trans_T_C = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rot_q_T_C = Eigen::Quaterniond::Identity();
};

// Nlohmann JSON serialization for CamFrame.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(CamFrame, timestamp, img_path,
                                                keypoints, trans_T_C,
                                                rot_q_T_C);

/// @brief Structure for a single IMU frame.
struct ImuFrame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<ImuFrame>;

  // Factory method to create a shared pointer instance.
  static Ptr Create() { return Ptr(new ImuFrame()); }

  // Timestamp in seconds.
  double timestamp = -1.;

  // Measurement of accelerometer and gyroscope.
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
};

/// @brief Structure for a single magnetometer frame.
struct MagFrame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<MagFrame>;

  // Factory method to create a shared pointer instance.
  static Ptr Create() { return Ptr(new MagFrame()); }

  // Timestamp in seconds.
  double timestamp = -1.;

  // Measurement of magnetometer.
  Eigen::Vector3d mag = Eigen::Vector3d::Zero();
};

/// @brief Structure for a single pose frame.
struct PoseFrame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<PoseFrame>;

  // Factory method to create a shared pointer instance.
  static Ptr Create() { return Ptr(new PoseFrame()); }

  // Timestamp in seconds.
  double timestamp = -1.;

  // Measurement of translation and rotation.
  Eigen::Vector3d trans = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rot_q = Eigen::Quaterniond::Identity();
};

// ===========================================================================
// =========================== Sensor sequence types =========================
// ===========================================================================

/// @brief Template structure for a sequence of sensor frames.
template <typename FrameT>
struct SensorSequence {
  using Ptr = std::shared_ptr<SensorSequence<FrameT>>;

  // Factory method to create a shared pointer instance.
  static Ptr Create() { return Ptr(new SensorSequence<FrameT>()); }

  // Container for multiple sensor frames.
  std::vector<FrameT> frames;

  // STL-like interface methods.
  void Add(const FrameT& f) { frames.push_back(f); }
  void Add(FrameT&& f) { frames.push_back(std::move(f)); }

  const FrameT& operator[](size_t i) const { return frames[i]; }
  FrameT& operator[](size_t i) { return frames[i]; }

  const FrameT& At(size_t i) const { return frames.at(i); }
  FrameT& At(size_t i) { return frames.at(i); }

  const FrameT& Front() const { return frames.front(); }
  FrameT& Front() { return frames.front(); }

  const FrameT& Back() const { return frames.back(); }
  FrameT& Back() { return frames.back(); }

  const std::vector<FrameT>& GetFrames() const { return frames; }

  auto begin() { return frames.begin(); }
  auto end() { return frames.end(); }
  auto begin() const { return frames.begin(); }
  auto end() const { return frames.end(); }
  auto cbegin() const { return frames.cbegin(); }
  auto cend() const { return frames.cend(); }

  auto insert(typename std::vector<FrameT>::const_iterator pos,
              const FrameT& f) {
    return frames.insert(pos, f);
  }

  auto insert(typename std::vector<FrameT>::const_iterator pos, FrameT&& f) {
    return frames.insert(pos, std::move(f));
  }

  template <class InputIt>
  auto insert(typename std::vector<FrameT>::const_iterator pos, InputIt first,
              InputIt last) {
    return frames.insert(pos, first, last);
  }

  auto erase(typename std::vector<FrameT>::const_iterator pos) {
    return frames.erase(pos);
  }

  auto erase(typename std::vector<FrameT>::const_iterator first,
             typename std::vector<FrameT>::const_iterator last) {
    return frames.erase(first, last);
  }

  size_t Size() const { return frames.size(); }
  bool Empty() const { return frames.empty(); }

  void Clear() { frames.clear(); }

 private:
  SensorSequence() = default;
};

// Sensor sequence types.
using CamSequence = SensorSequence<CamFrame::Ptr>;
using ImuSequence = SensorSequence<ImuFrame::Ptr>;
using MagSequence = SensorSequence<MagFrame::Ptr>;
using PoseSequence = SensorSequence<PoseFrame::Ptr>;

// Custom Nlohmann JSON serialization for CamSequence.
inline void to_json(nlohmann::json& j, const CamSequence& seq) {
  j["frames"] = nlohmann::json::array();
  for (const auto& frame_ptr : seq.frames) {
    if (frame_ptr) {
      j["frames"].push_back(*frame_ptr);
    } else {
      j["frames"].push_back(nullptr);
    }
  }
}

// Custom Nlohmann JSON serialization for CamSequence.
inline void from_json(const nlohmann::json& j, CamSequence& seq) {
  seq.frames.clear();
  if (j.contains("frames")) {
    for (const auto& item : j.at("frames")) {
      if (item.is_null()) {
        seq.frames.push_back(nullptr);
      } else {
        auto frame = CamFrame::Create();
        item.get_to(*frame);
        seq.frames.push_back(frame);
      }
    }
  }
}

// ===========================================================================
// ============================= Data Loader Base ============================
// ===========================================================================

/// @brief Base class for sensor data loaders.
class DataLoaderBase {
 public:
  using Ptr = std::shared_ptr<DataLoaderBase>;

  virtual ~DataLoaderBase() = default;

  // Virtual method to load sensor data from a file.
  virtual bool Load(const std::string& data_path) = 0;
};

}  // namespace xr_ucalib
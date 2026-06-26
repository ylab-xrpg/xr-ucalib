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
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <nlohmann/json.hpp>
#include <sophus/se3.hpp>

#include <xr_ucalib/uc_common/config/types.h>
// clang-format on

// Template specialization for commonly used types.
namespace nlohmann {

// ===========================================================================
// ================================ Enum types ===============================
// ===========================================================================

template <>
struct adl_serializer<xr_ucalib::CamModelType> {
  static void to_json(json& j, const xr_ucalib::CamModelType& m) {
    j = xr_ucalib::CamModelTypeToString(m);
  }

  static void from_json(const json& j, xr_ucalib::CamModelType& m) {
    std::string str = j.get<std::string>();
    try {
      m = xr_ucalib::CamModelTypeFromString(str);
    } catch (...) {
      m = xr_ucalib::CamModelType::INVALID;
    }
  }
};

template <>
struct adl_serializer<xr_ucalib::ImuModelType> {
  static void to_json(json& j, const xr_ucalib::ImuModelType& m) {
    j = xr_ucalib::ImuModelTypeToString(m);
  }

  static void from_json(const json& j, xr_ucalib::ImuModelType& m) {
    std::string str = j.get<std::string>();
    try {
      m = xr_ucalib::ImuModelTypeFromString(str);
    } catch (...) {
      m = xr_ucalib::ImuModelType::INVALID;
    }
  }
};

template <>
struct adl_serializer<xr_ucalib::FiducialType> {
  static void to_json(json& j, const xr_ucalib::FiducialType& m) {
    j = xr_ucalib::FiducialTypeToString(m);
  }
  static void from_json(const json& j, xr_ucalib::FiducialType& m) {
    std::string str = j.get<std::string>();
    try {
      m = xr_ucalib::FiducialTypeFromString(str);
    } catch (...) {
      m = xr_ucalib::FiducialType::INVALID;
    }
  }
};

template <>
struct adl_serializer<xr_ucalib::DetectionDisplayMode> {
  static void to_json(json& j, const xr_ucalib::DetectionDisplayMode& m) {
    j = xr_ucalib::DetectionDisplayModeToString(m);
  }
  static void from_json(const json& j, xr_ucalib::DetectionDisplayMode& m) {
    std::string str = j.get<std::string>();
    try {
      m = xr_ucalib::DetectionDisplayModeFromString(str);
    } catch (...) {
      m = xr_ucalib::DetectionDisplayMode::NONE;
    }
  }
};

// ===========================================================================
// ============================ Eigen/Sophus types ===========================
// ===========================================================================

template <>
struct adl_serializer<Eigen::Vector2d> {
  static void to_json(json& j, const Eigen::Vector2d& v) {
    j = json{{"x", v.x()}, {"y", v.y()}};
  }

  static void from_json(const json& j, Eigen::Vector2d& v) {
    v.x() = j.at("x").get<double>();
    v.y() = j.at("y").get<double>();
  }
};

template <>
struct adl_serializer<Eigen::Vector3d> {
  static void to_json(json& j, const Eigen::Vector3d& v) {
    j = json{{"x", v.x()}, {"y", v.y()}, {"z", v.z()}};
  }

  static void from_json(const json& j, Eigen::Vector3d& v) {
    v.x() = j.at("x").get<double>();
    v.y() = j.at("y").get<double>();
    v.z() = j.at("z").get<double>();
  }
};

template <>
struct adl_serializer<Eigen::Vector4d> {
  static void to_json(json& j, const Eigen::Vector4d& v) {
    j = json{
        {"acc_n", v[0]}, {"acc_b", v[1]}, {"gyr_n", v[2]}, {"gyr_b", v[3]}};
  }

  static void from_json(const json& j, Eigen::Vector4d& v) {
    v[0] = j.at("acc_n").get<double>();
    v[1] = j.at("acc_b").get<double>();
    v[2] = j.at("gyr_n").get<double>();
    v[3] = j.at("gyr_b").get<double>();
  }
};

template <>
struct adl_serializer<Eigen::Quaterniond> {
  static void to_json(json& j, const Eigen::Quaterniond& q) {
    j = json{{"x", q.x()}, {"y", q.y()}, {"z", q.z()}, {"w", q.w()}};
  }

  static void from_json(const json& j, Eigen::Quaterniond& q) {
    q.x() = j.at("x").get<double>();
    q.y() = j.at("y").get<double>();
    q.z() = j.at("z").get<double>();
    q.w() = j.at("w").get<double>();
  }
};

template <>
struct adl_serializer<Eigen::Matrix3d> {
  static void to_json(json& j, const Eigen::Matrix3d& m) {
    j = json::array();
    for (int i = 0; i < 3; ++i) {
      j.push_back({m(i, 0), m(i, 1), m(i, 2)});
    }
  }

  static void from_json(const json& j, Eigen::Matrix3d& m) {
    for (int i = 0; i < 3; ++i) {
      m(i, 0) = j.at(i).at(0).get<double>();
      m(i, 1) = j.at(i).at(1).get<double>();
      m(i, 2) = j.at(i).at(2).get<double>();
    }
  }
};

template <>
struct adl_serializer<Sophus::SO3d> {
  static void to_json(json& j, const Sophus::SO3d& so3) {
    Eigen::Quaterniond q = so3.unit_quaternion();
    j = json{{"x", q.x()}, {"y", q.y()}, {"z", q.z()}, {"w", q.w()}};
  }

  static void from_json(const json& j, Sophus::SO3d& so3) {
    Eigen::Quaterniond q;
    q.x() = j.at("x").get<double>();
    q.y() = j.at("y").get<double>();
    q.z() = j.at("z").get<double>();
    q.w() = j.at("w").get<double>();
    so3 = Sophus::SO3d(q);
  }
};

template <>
struct adl_serializer<Sophus::Matrix<double, 10, 1>> {
  static void to_json(json& j, const Sophus::Matrix<double, 10, 1>& v) {
    j = json::array();
    for (int i = 0; i < 10; ++i) {
      j.push_back(v(i));
    }
  }

  static void from_json(const json& j, Sophus::Matrix<double, 10, 1>& v) {
    for (int i = 0; i < 10; ++i) {
      v(i) = j.at(i).get<double>();
    }
  }
};

// ===========================================================================
// ================================ Map types ================================
// ===========================================================================

template <>
struct adl_serializer<std::map<std::string, double>> {
  static void to_json(json& j, const std::map<std::string, double>& m) {
    j = json::object();
    for (const auto& [key, val] : m) {
      j[key] = val;
    }
  }

  static void from_json(const json& j, std::map<std::string, double>& m) {
    m.clear();
    for (auto it = j.begin(); it != j.end(); ++it) {
      m[it.key()] = it.value().get<double>();
    }
  }
};

template <>
struct adl_serializer<std::map<int, Eigen::Vector2d>> {
  static void to_json(json& j, const std::map<int, Eigen::Vector2d>& m) {
    j = json::object();
    for (const auto& [id, pt] : m) {
      j[std::to_string(id)] = json{{"x", pt.x()}, {"y", pt.y()}};
    }
  }

  static void from_json(const json& j, std::map<int, Eigen::Vector2d>& m) {
    m.clear();
    for (auto it = j.begin(); it != j.end(); ++it) {
      int id = std::stoi(it.key());
      Eigen::Vector2d pt;
      pt.x() = it.value().at("x").get<double>();
      pt.y() = it.value().at("y").get<double>();
      m[id] = pt;
    }
  }
};

template <>
struct adl_serializer<std::map<int, Sophus::Vector3d>> {
  static void to_json(json& j, const std::map<int, Sophus::Vector3d>& m) {
    j = json::object();
    for (const auto& [id, val] : m) {
      j[std::to_string(id)] =
          json{{"x", val.x()}, {"y", val.y()}, {"z", val.z()}};
    }
  }

  static void from_json(const json& j, std::map<int, Sophus::Vector3d>& m) {
    m.clear();
    for (auto it = j.begin(); it != j.end(); ++it) {
      int id = std::stoi(it.key());
      Sophus::Vector3d v;
      v.x() = it.value().at("x").get<double>();
      v.y() = it.value().at("y").get<double>();
      v.z() = it.value().at("z").get<double>();
      m[id] = v;
    }
  }
};

template <>
struct adl_serializer<std::map<std::string, Sophus::Vector3d>> {
  static void to_json(json& j,
                      const std::map<std::string, Sophus::Vector3d>& m) {
    j = json::object();
    for (const auto& [key, val] : m) {
      j[key] = json{{"x", val.x()}, {"y", val.y()}, {"z", val.z()}};
    }
  }

  static void from_json(const json& j,
                        std::map<std::string, Sophus::Vector3d>& m) {
    m.clear();
    for (auto it = j.begin(); it != j.end(); ++it) {
      Sophus::Vector3d v;
      v.x() = it.value().at("x").get<double>();
      v.y() = it.value().at("y").get<double>();
      v.z() = it.value().at("z").get<double>();
      m[it.key()] = v;
    }
  }
};

template <>
struct adl_serializer<std::map<int, Sophus::SO3d>> {
  static void to_json(json& j, const std::map<int, Sophus::SO3d>& m) {
    j = json::object();
    for (const auto& [id, val] : m) {
      Eigen::Quaterniond q = val.unit_quaternion();
      j[std::to_string(id)] =
          json{{"x", q.x()}, {"y", q.y()}, {"z", q.z()}, {"w", q.w()}};
    }
  }

  static void from_json(const json& j, std::map<int, Sophus::SO3d>& m) {
    m.clear();
    for (auto it = j.begin(); it != j.end(); ++it) {
      int id = std::stoi(it.key());
      Eigen::Quaterniond q;
      q.x() = it.value().at("x").get<double>();
      q.y() = it.value().at("y").get<double>();
      q.z() = it.value().at("z").get<double>();
      q.w() = it.value().at("w").get<double>();
      m[id] = Sophus::SO3d(q);
    }
  }
};

template <>
struct adl_serializer<std::map<std::string, Sophus::SO3d>> {
  static void to_json(json& j, const std::map<std::string, Sophus::SO3d>& m) {
    j = json::object();
    for (const auto& [key, val] : m) {
      Eigen::Quaterniond q = val.unit_quaternion();
      j[key] = json{{"x", q.x()}, {"y", q.y()}, {"z", q.z()}, {"w", q.w()}};
    }
  }

  static void from_json(const json& j, std::map<std::string, Sophus::SO3d>& m) {
    m.clear();
    for (auto it = j.begin(); it != j.end(); ++it) {
      Eigen::Quaterniond q;
      q.x() = it.value().at("x").get<double>();
      q.y() = it.value().at("y").get<double>();
      q.z() = it.value().at("z").get<double>();
      q.w() = it.value().at("w").get<double>();
      m[it.key()] = Sophus::SO3d(q);
    }
  }
};

}  // namespace nlohmann
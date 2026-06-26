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

#include "xr_ucalib/uc_common/sensor_data/mag_data_loader.h"

// clang-format off
#include <fstream>
#include <sstream>

#include <spdlog/spdlog.h>
// clang-format on

namespace xr_ucalib {

bool MagDataLoader::Load(const std::string &data_path) {
  // Create an empty magnetometer sequence.
  sequence_ = MagSequence::Create();

  std::ifstream file(data_path);
  if (!file.is_open()) {
    spdlog::error("Failed to open magnetometer data file: {}", data_path);
    return false;
  }

  std::string line;
  double last_timestamp = std::numeric_limits<double>::lowest();

  // Skip the first line (assumed to be header)
  std::getline(file, line);

  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string value;
    std::vector<double> values;

    constexpr const char *kMagFormatError =
        "Each line in the magnetometer file must be in format "
        "\"timestamp (ns), mx, my, mz\".";
    try {
      while (std::getline(ss, value, ',')) {
        values.push_back(std::stod(value));
      }
    } catch (const std::exception &e) {
      spdlog::error(kMagFormatError);
      return false;
    }

    if (values.size() == 4) {
      // Convert ns to s.
      values[0] /= 1e9;

      double timestamp(values[0]);
      Eigen::Vector3d mag(values[1], values[2], values[3]);

      double mag_norm = mag.norm();
      constexpr double kMagNormThreshold = 0.5;
      if (std::abs(mag_norm - 1.0) > kMagNormThreshold / 2.) {
        spdlog::warn(
            "Magnetometer data at {:.9f} is not close to unit vector "
            "(norm={:.4f}). Please ensure input data is intrinsically "
            "calibrated.",
            timestamp, mag_norm);

        if (std::abs(mag_norm - 1.0) > kMagNormThreshold) {
          spdlog::error(
              "Magnetometer data at {:.9f} is not close to unit vector "
              "(norm={:.4f}). Please ensure input data is intrinsically "
              "calibrated.",
              timestamp, mag_norm);
          return false;
        }
      }
      mag.normalize();

      // Check for strictly increasing timestamps.
      if (timestamp <= last_timestamp) {
        spdlog::error("Magnetometer timestamps are not strictly increasing.");
        return false;
      }

      // Add the IMU frame to the sequence.
      MagFrame::Ptr mag_frame = MagFrame::Create();
      mag_frame->timestamp = timestamp;
      mag_frame->mag = mag;

      sequence_->Add(mag_frame);
      last_timestamp = timestamp;
    } else {
      spdlog::error(kMagFormatError);
      return false;
    }
  }

  return true;
}

}  // namespace xr_ucalib
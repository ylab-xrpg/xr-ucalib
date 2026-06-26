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

#include "xr_ucalib/uc_common/sensor_data/imu_data_loader.h"

// clang-format off
#include <fstream>
#include <sstream>

#include <spdlog/spdlog.h>
// clang-format on

namespace xr_ucalib {

bool ImuDataLoader::Load(const std::string &data_path) {
  // Create an empty IMU sequence.
  sequence_ = ImuSequence::Create();

  sequence_->Clear();

  std::ifstream file(data_path);
  if (!file.is_open()) {
    spdlog::error("Failed to open imu data file: {}", data_path);
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

    constexpr const char *kImuFormatError =
        "Each line in the IMU file must be in format "
        "\"timestamp (ns), wx (rad/s), wy (rad/s), wz (rad/s), "
        "ax (m/s^2), ay (m/s^2), az (m/s^2)\".";
    try {
      while (std::getline(ss, value, ',')) {
        values.push_back(std::stod(value));
      }
    } catch (const std::exception &e) {
      spdlog::error(kImuFormatError);
      return false;
    }

    if (values.size() == 7) {
      // Convert ns to s.
      values[0] /= 1e9;

      double timestamp(values[0]);
      Eigen::Vector3d acc(values[4], values[5], values[6]);
      Eigen::Vector3d gyr(values[1], values[2], values[3]);

      // Check for strictly increasing timestamps.
      if (timestamp <= last_timestamp) {
        spdlog::error("IMU timestamps are not strictly increasing.");
        return false;
      }

      // Add the IMU frame to the sequence.
      ImuFrame::Ptr imu_frame = ImuFrame::Create();
      imu_frame->timestamp = timestamp;
      imu_frame->acc = acc;
      imu_frame->gyr = gyr;

      sequence_->Add(imu_frame);
      last_timestamp = timestamp;
    } else {
      spdlog::error(kImuFormatError);
      return false;
    }
  }

  return true;
}

}  // namespace xr_ucalib
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

#include <memory>
#include <string>

#include "xr_ucalib/uc_common/sensor_data/sensor_data_types.h"

namespace xr_ucalib {

/// @brief IMU data loader class.
class ImuDataLoader : public DataLoaderBase {
 public:
  using Ptr = std::shared_ptr<ImuDataLoader>;

  // Factory method to create a shared pointer instance.
  static Ptr Create() { return Ptr(new ImuDataLoader()); }

  // Override Load method to load IMU data from file.
  bool Load(const std::string& data_path) override;

  // Get the loaded IMU sequence.
  ImuSequence::Ptr GetSequence() const { return sequence_; };

 private:
  ImuDataLoader() = default;

  ImuSequence::Ptr sequence_;
};

}  // namespace xr_ucalib

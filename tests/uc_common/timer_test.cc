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
#include <gtest/gtest.h>

#include "xr_ucalib/uc_common/utils/timer.h"
// clang-format on

namespace xr_ucalib {
namespace {

TEST(TimerTest, Default) {
  Timer timer;
  EXPECT_EQ(timer.ElapsedMicroSeconds(), 0);
  EXPECT_EQ(timer.ElapsedSeconds(), 0);
  EXPECT_EQ(timer.ElapsedMinutes(), 0);
  EXPECT_EQ(timer.ElapsedHours(), 0);
}

TEST(TimerTest, Start) {
  Timer timer;
  timer.Start();
  EXPECT_GE(timer.ElapsedMicroSeconds(), 0);
  EXPECT_GE(timer.ElapsedSeconds(), 0);
  EXPECT_GE(timer.ElapsedMinutes(), 0);
  EXPECT_GE(timer.ElapsedHours(), 0);
}

TEST(TimerTest, Pause) {
  Timer timer;
  timer.Start();
  timer.Pause();
  double prev_time = timer.ElapsedMicroSeconds();
  for (size_t i = 0; i < 1000; ++i) {
    EXPECT_EQ(timer.ElapsedMicroSeconds(), prev_time);
    prev_time = timer.ElapsedMicroSeconds();
  }
  timer.Resume();
  for (size_t i = 0; i < 1000; ++i) {
    EXPECT_GE(timer.ElapsedMicroSeconds(), prev_time);
  }
  timer.Reset();
  EXPECT_EQ(timer.ElapsedMicroSeconds(), 0);
}

}  // namespace
}  // namespace xr_ucalib
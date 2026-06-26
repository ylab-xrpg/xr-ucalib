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

#include <chrono>

namespace xr_ucalib {

/**
 * @brief Simple high-resolution timer with start, pause, resume, and reset.
 *
 * Provides microsecond-precision time measurement. Elapsed time can be queried
 * in microseconds, seconds, minutes, or hours. The timer does not run until
 * Start() is called.
 */
class Timer {
 public:
  Timer();

  // Starts the timer if not started yet; Restart() resets and starts.
  void Start();
  void Restart();

  // Pauses or resumes the timer.
  void Pause();
  void Resume();

  // Resets the timer to zero and stops it.
  void Reset();

  // Returns elapsed time in different units.
  double ElapsedMicroSeconds() const;
  double ElapsedSeconds() const;
  double ElapsedMinutes() const;
  double ElapsedHours() const;

  // Prints elapsed time in seconds, minutes, or hours.
  void PrintSeconds() const;
  void PrintMinutes() const;
  void PrintHours() const;

 private:
  // Whether the timer has been started.
  bool started_;
  // Whether the timer is currently paused.
  bool paused_;

  // Timestamp when the timer was last started or resumed.
  std::chrono::high_resolution_clock::time_point start_time_;

  // Timestamp when the timer was paused.
  std::chrono::high_resolution_clock::time_point pause_time_;
};

}  // namespace xr_ucalib
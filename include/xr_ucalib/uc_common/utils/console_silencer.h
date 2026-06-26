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

namespace xr_ucalib {

/**
 * @brief A RAII-style utility class to suppress console output (stdout and
 * stderr).
 *
 * This class redirects stdout and stderr to /dev/null upon construction and
 * restores them to their original state upon destruction. 
 * 
 * Usage: Create an instance of ConsoleSilencer in a scope where you want to
 * silence console output.
 * {
 *   ConsoleSilencer silencer;
 *  // Code that produces console output.
 * }
 */
class ConsoleSilencer {
 public:
  /**
   * @brief Construct a new Console Silencer object.
   *
   * Redirects stdout and stderr to /dev/null.
   */
  ConsoleSilencer();

  /**
   * @brief Destroy the Console Silencer object.
   *
   * Restores stdout and stderr to their original state.
   */
  ~ConsoleSilencer();

  // Disable copy and move semantics to prevent resource management issues.
  ConsoleSilencer(const ConsoleSilencer&) = delete;
  ConsoleSilencer& operator=(const ConsoleSilencer&) = delete;
  ConsoleSilencer(ConsoleSilencer&&) = delete;
  ConsoleSilencer& operator=(ConsoleSilencer&&) = delete;

 private:
  int original_stdout_fd_;
  int original_stderr_fd_;
  int null_fd_;
#ifdef _WIN32
  void* original_stdout_handle_ = nullptr;
  void* original_stderr_handle_ = nullptr;
#endif
};

}  // namespace xr_ucalib

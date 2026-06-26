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

#include "xr_ucalib/uc_common/utils/console_silencer.h"

#include <fcntl.h>

#ifdef _WIN32
#include <io.h>
#include <cstdio>
#include <windows.h>
#define dup _dup
#define dup2 _dup2
#define fileno _fileno
#define open _open
#define close _close
#ifndef STDOUT_FILENO
#define STDOUT_FILENO 1
#endif
#ifndef STDERR_FILENO
#define STDERR_FILENO 2
#endif
#ifndef O_WRONLY
#define O_WRONLY _O_WRONLY
#endif
#else
#include <unistd.h>
#endif

#include <iostream>

namespace xr_ucalib {

ConsoleSilencer::ConsoleSilencer() {
  // Flush standard streams to ensure all pending output is written before
  // redirection.
  std::cout.flush();
  std::cerr.flush();
  std::fflush(stdout);
  std::fflush(stderr);

  // Duplicate the file descriptors for stdout and stderr to save them.
  original_stdout_fd_ = dup(STDOUT_FILENO);
  original_stderr_fd_ = dup(STDERR_FILENO);

#ifdef _WIN32
  // Save the Windows process-level standard handles.
  original_stdout_handle_ = GetStdHandle(STD_OUTPUT_HANDLE);
  original_stderr_handle_ = GetStdHandle(STD_ERROR_HANDLE);
  if (original_stdout_handle_ != INVALID_HANDLE_VALUE &&
      original_stdout_handle_ != nullptr) {
    SetHandleInformation(original_stdout_handle_,
                         HANDLE_FLAG_PROTECT_FROM_CLOSE,
                         HANDLE_FLAG_PROTECT_FROM_CLOSE);
  }
  if (original_stderr_handle_ != INVALID_HANDLE_VALUE &&
      original_stderr_handle_ != nullptr) {
    SetHandleInformation(original_stderr_handle_,
                         HANDLE_FLAG_PROTECT_FROM_CLOSE,
                         HANDLE_FLAG_PROTECT_FROM_CLOSE);
  }
#endif

  // Open the null device for writing (platform-specific).
#ifdef _WIN32
  null_fd_ = open("nul", O_WRONLY);
#else
  null_fd_ = open("/dev/null", O_WRONLY);
#endif

  // Redirect stdout and stderr to the null device.
  if (null_fd_ != -1) {
    dup2(null_fd_, STDOUT_FILENO);
    dup2(null_fd_, STDERR_FILENO);
  }
}

ConsoleSilencer::~ConsoleSilencer() {
  // Restore original stdout and stderr first.
  if (original_stdout_fd_ != -1) {
    dup2(original_stdout_fd_, STDOUT_FILENO);
    close(original_stdout_fd_);
  }

  if (original_stderr_fd_ != -1) {
    dup2(original_stderr_fd_, STDERR_FILENO);
    close(original_stderr_fd_);
  }

  // Close the file descriptor for the null device.
  if (null_fd_ != -1) {
    close(null_fd_);
  }

#ifdef _WIN32
  // Remove the close-protection we set in the constructor.
  if (original_stdout_handle_ != INVALID_HANDLE_VALUE &&
      original_stdout_handle_ != nullptr) {
    SetHandleInformation(original_stdout_handle_,
                         HANDLE_FLAG_PROTECT_FROM_CLOSE, 0);
  }
  if (original_stderr_handle_ != INVALID_HANDLE_VALUE &&
      original_stderr_handle_ != nullptr) {
    SetHandleInformation(original_stderr_handle_,
                         HANDLE_FLAG_PROTECT_FROM_CLOSE, 0);
  }
  // Restore the original Windows process-level standard handles.
  SetStdHandle(STD_OUTPUT_HANDLE, original_stdout_handle_);
  SetStdHandle(STD_ERROR_HANDLE, original_stderr_handle_);
#endif

  std::cout.flush();
  std::cerr.flush();
}

}  // namespace xr_ucalib

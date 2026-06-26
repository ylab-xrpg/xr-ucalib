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
#include "xr_ucalib/uc_common/sensor_data/fiducial_detector_base.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
// clang-format on

namespace xr_ucalib {

bool FiducialDetecterBase::DetectSequence(const std::string &seq_dir,
                                          CamSequence::Ptr &cam_seq, int &width,
                                          int &height) {
  std::vector<std::string> img_files;
  // Step 1: Gather valid image files.
  for (const auto &entry : std::filesystem::directory_iterator(seq_dir)) {
    if (!entry.is_regular_file()) {
      spdlog::warn("Skipping non-regular file: {}",
                   entry.path().filename().string());
      continue;
    }

    std::string file_path = entry.path().string();
    std::string extension = entry.path().extension().string();

    // Convert extension to lowercase for case-insensitive comparison.
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    // Check if the file has a valid image extension.
    if (std::find(kValidImageExtensions.begin(), kValidImageExtensions.end(),
                  extension) != kValidImageExtensions.end()) {
      img_files.push_back(file_path);
    } else {
      spdlog::warn("Skipping non-image file: {}",
                   entry.path().filename().string());
    }
  }

  if (img_files.empty()) {
    spdlog::error("No valid image files found in directory: {}", seq_dir);
    return false;
  }

  std::sort(img_files.begin(), img_files.end());

  size_t total_imgs = img_files.size();
  size_t processed_imgs = 0;

  width = -1;
  height = -1;

  // ============================================================================

  // Step2: Process each image file to detect fiducial markers.
  for (const auto &img_file : img_files) {
    processed_imgs++;
    if (processed_imgs % 10 == 0 || processed_imgs == total_imgs) {
      std::printf("\rDetecting fiducial targets in images: %zu/%zu",
                  processed_imgs, total_imgs);
      std::fflush(stdout);
    }

    cv::Mat img = cv::imread(img_file, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
      std::printf("\n");
      spdlog::warn("Failed to read image: {}", img_file);
      continue;
    }

    if (width == -1 && height == -1) {
      width = img.cols;
      height = img.rows;
    } else {
      if (width != img.cols || height != img.rows) {
        std::printf("\n");
        spdlog::error(
            "Image size mismatch! Expected: {}x{}, Actual: {}x{}. File: {}",
            width, height, img.cols, img.rows, img_file);
        return false;
      }
    }

    CamFrame::Ptr cam_frame = CamFrame::Create();
    if (DetectFrame(img, cam_frame)) {
      double timestamp;
      if (!Path2Time(img_file, timestamp)) {
        std::printf("\n");
        spdlog::error("Failed to parse timestamp from {}", img_file);
        continue;
      }

      cam_frame->timestamp = timestamp;
      cam_frame->img_path = img_file;

      cam_seq->Add(cam_frame);
    } else {
      spdlog::debug("Failed to detect fiducial markers in image: {}", img_file);
    }
  }
  std::printf("\n");

  // ============================================================================

  std::sort(cam_seq->begin(), cam_seq->end(),
            [](const CamFrame::Ptr &a, const CamFrame::Ptr &b) {
              return a->timestamp < b->timestamp;
            });

  return true;
}

bool FiducialDetecterBase::DetectSequenceMT(const std::string &seq_dir,
                                            CamSequence::Ptr &cam_seq,
                                            int &width, int &height,
                                            const int num_threads) {
  std::vector<std::string> img_files;
  // Step 1: Gather valid image files.
  for (const auto &entry : std::filesystem::directory_iterator(seq_dir)) {
    if (!entry.is_regular_file()) {
      spdlog::warn("Skipping non-regular file: {}",
                   entry.path().filename().string());
      continue;
    }

    std::string file_path = entry.path().string();
    std::string extension = entry.path().extension().string();

    // Convert extension to lowercase for case-insensitive comparison.
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    // Check if the file has a valid image extension.
    if (std::find(kValidImageExtensions.begin(), kValidImageExtensions.end(),
                  extension) != kValidImageExtensions.end()) {
      img_files.push_back(file_path);
    } else {
      spdlog::warn("Skipping non-image file: {}",
                   entry.path().filename().string());
    }
  }

  if (img_files.empty()) {
    spdlog::error("No valid image files found in directory: {}", seq_dir);
    return false;
  }

  // ============================================================================

  // Step 2: Parallel processing for each image file to detect fiducial markers.
  size_t total_imgs = img_files.size();
  std::atomic<size_t> processed_imgs(0);
  std::atomic<bool> size_mismatch(false);
  width = -1;
  height = -1;

  // Step 2.1: Define worker function for parallel processing.
  std::mutex cam_seq_mutex;
  std::mutex io_mutex;
  std::mutex size_mutex;
  auto worker_func = [&](int start_idx, int end_idx) {
    for (int i = start_idx; i < end_idx; ++i) {
      if (size_mismatch) continue;

      const auto &img_file = img_files[i];
      size_t current_processed = ++processed_imgs;
      if (current_processed % 10 == 0 || current_processed == total_imgs) {
        std::lock_guard<std::mutex> lock(io_mutex);
        std::printf("\rDetecting fiducial targets in images: %zu/%zu",
                    current_processed, total_imgs);
        std::fflush(stdout);
      }

      cv::Mat image = cv::imread(img_file, cv::IMREAD_GRAYSCALE);
      if (image.empty()) {
        std::lock_guard<std::mutex> lock(io_mutex);
        std::printf("\n");
        spdlog::warn("Failed to read image: {}", img_file);
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(size_mutex);
        if (width == -1 && height == -1) {
          width = image.cols;
          height = image.rows;
        } else {
          if (width != image.cols || height != image.rows) {
            std::lock_guard<std::mutex> lk(io_mutex);
            std::printf("\n");
            spdlog::error(
                "Image size mismatch! Expected: {}x{}, Actual: {}x{}. File: {}",
                width, height, image.cols, image.rows, img_file);
            size_mismatch = true;
          }
        }
      }

      if (size_mismatch) continue;

      CamFrame::Ptr cam_frame = CamFrame::Create();
      if (DetectFrame(image, cam_frame)) {
        double timestamp;
        if (!Path2Time(img_file, timestamp)) {
          std::lock_guard<std::mutex> lock(io_mutex);
          std::printf("\n");
          spdlog::error("Failed to parse timestamp from {}", img_file);
          continue;
        }

        cam_frame->timestamp = timestamp;
        cam_frame->img_path = img_file;

        std::lock_guard<std::mutex> lock(cam_seq_mutex);

        cam_seq->Add(cam_frame);
      } else {
        spdlog::debug("Failed to detect fiducial markers in image: {}",
                      img_file);
      }
    }
  };

  // Step 2.2: Launch threads for parallel processing.
  const int kMultiThreadNum = std::max(1, num_threads);
  std::vector<std::thread> thread_pool;
  int task_step = (total_imgs + kMultiThreadNum - 1) / kMultiThreadNum;

  for (int i = 0; i < kMultiThreadNum; ++i) {
    int start = i * task_step;
    int end = std::min(start + task_step, static_cast<int>(total_imgs));
    if (start < end) {
      thread_pool.emplace_back(worker_func, start, end);
    }
  }

  for (auto &thread : thread_pool) {
    thread.join();
  }
  std::printf("\n");

  // ============================================================================

  if (size_mismatch) return false;

  std::sort(cam_seq->begin(), cam_seq->end(),
            [](const CamFrame::Ptr &a, const CamFrame::Ptr &b) {
              return a->timestamp < b->timestamp;
            });

  return true;
}

bool FiducialDetecterBase::Path2Time(std::string img_path, double &timestamp) {
  auto ReplaceAll = [](std::string &str, const std::string &from,
                       const std::string &to) {
    if (from.empty()) return;
    size_t start_pos = 0;
    while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
      str.replace(start_pos, from.length(), to);
      start_pos += to.length();
    }
  };

  ReplaceAll(img_path, "\\", "/");

  size_t last_slash = img_path.find_last_of('/');
  size_t last_dot = img_path.find_last_of('.');

  std::string name;

  if (last_slash != std::string::npos) {
    name = img_path.substr(last_slash + 1, last_dot - last_slash - 1);
  } else {
    name = img_path.substr(0, last_dot);
  }

  // Check if the filename is empty.
  if (name.empty()) {
    spdlog::warn("Failed to extract filename from path: {}", img_path);
    return false;
  }

  // Check if the filename contains only digits.
  if (name.find_first_not_of("0123456789") != std::string::npos) {
    spdlog::warn("Filename contains non-digit characters: {}", name);
    return false;
  }

  try {
    uint64_t nano_sec = std::stoull(name);
    timestamp = nano_sec * 1e-9;
    return true;
  } catch (const std::invalid_argument &e) {
    spdlog::warn("Invalid timestamp format in filename: {}", name);
    return false;
  } catch (const std::out_of_range &e) {
    spdlog::warn("Timestamp value out of range in filename: {}", name);
    return false;
  }
}

}  // namespace xr_ucalib
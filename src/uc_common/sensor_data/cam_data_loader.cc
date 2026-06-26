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
#include "xr_ucalib/uc_common/sensor_data/cam_data_loader.h"

#include <filesystem>
#include <fstream>

#include <spdlog/spdlog.h>

#include "xr_ucalib/uc_common/sensor_data/apriltag_detector.h"
#include "xr_ucalib/uc_common/utils/timer.h"
// clang-format on

namespace xr_ucalib {

bool CamDataLoader::Load(const std::string& data_path) {
  // Create an empty camera sequence.
  sequence_ = CamSequence::Create();

  if (!std::filesystem::exists(data_path) ||
      !std::filesystem::is_directory(data_path)) {
    spdlog::error("Data path does not exist or is not a directory: {}",
                  data_path);
    return false;
  }

  FiducialDetecterBase::Ptr detector;
  if (fiducial_type_ == FiducialType::APRILTAG) {
    detector = AprilTagDetecter::Create();
  } else {
    spdlog::error("Fiducial type not set or unsupported.");
    return false;
  }

  if (!detector->DetectSequenceMT(data_path, sequence_, img_width_, img_height_,
                                  num_threads_)) {
    spdlog::error("Failed to load camera data from: {}", data_path);
    return false;
  }

  return true;
}

bool CamDataLoader::ShowDetections(bool step_mode) {
  if (!sequence_ || sequence_->Size() == 0) {
    spdlog::warn("No camera data to show. Please load data first.");
    return false;
  }

  Timer timer;
  for (size_t i = 0; i < sequence_->Size(); ++i) {
    timer.Start();

    CamFrame::Ptr frame = sequence_->At(i);

    // Load image.
    cv::Mat img = cv::imread(frame->img_path, cv::IMREAD_COLOR);
    if (img.empty()) {
      spdlog::warn("Failed to read image for showing detections: {}",
                   frame->img_path);
      return false;
    }

    // Calculate dynamic parameters based on image size
    int image_size = std::min(img.rows, img.cols);
    int circle_radius = std::max(4, static_cast<int>(image_size * 0.006));
    int circle_thickness = std::max(2, static_cast<int>(image_size * 0.002));
    double font_scale = std::max(0.4, image_size * 0.0005);
    int font_thickness = std::max(1, static_cast<int>(image_size * 0.002));

    // Draw keypoints and IDs.
    for (const auto& [id, kp] : frame->keypoints) {
      cv::Point2d keypoint(kp.x(), kp.y());

      cv::circle(img, keypoint, circle_radius, cv::Scalar(255),
                 circle_thickness);
      cv::putText(img, std::to_string(id),
                  cv::Point2d(keypoint.x + 5, keypoint.y - 5),
                  cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 255, 0),
                  font_thickness);
    }

    // Resize image if needed, ensuring max height is 1080 to fit screen.
    constexpr int kMaxHeight = 1080;
    double scale = 1.0;
    if (img.rows > kMaxHeight) {
      scale = static_cast<double>(kMaxHeight) / img.rows;
      int kNewWidth = static_cast<int>(img.cols * scale);
      cv::resize(img, img, cv::Size(kNewWidth, kMaxHeight), 0, 0,
                 cv::INTER_LINEAR);
    }

    // Show the image sequence at a specified frame rate.
    std::string filename =
        std::filesystem::path(frame->img_path).filename().string();
    std::string display_title =
        "Camera Detections for " + filename + " (may be resized)";
    const std::string kWindowName = "Camera Detections";
    
    cv::imshow(kWindowName, img);
    cv::setWindowTitle(kWindowName, display_title);

    double time_elapsed = timer.ElapsedSeconds() * 1000.0;  // in milliseconds

    double current_timestamp = frame->timestamp;
    double next_timestamp = (i + 1 < sequence_->Size())
                                ? sequence_->At(i + 1)->timestamp
                                : current_timestamp + 0.1;

    double wait_time_ms =
        (next_timestamp - current_timestamp) * 1000.0 - time_elapsed;

    if (step_mode) {
      int key = cv::waitKey(0);
      if (key == 27 || key == 'q' || key == 'Q') {  // ESC or q/Q to exit
        spdlog::info("ESC or q/Q pressed. Exiting visualization.");
        break;
      }
    } else {
      if (wait_time_ms < 0.) {
        wait_time_ms = 1.;
      }

      int key = cv::waitKey(wait_time_ms);
      if (key == 27 || key == 'q' || key == 'Q') {  // ESC or q/Q to exit
        spdlog::info("ESC or q/Q pressed. Exiting visualization.");
        break;
      }
      if (key == 'p' || key == 'P') {
        spdlog::info("Paused. Press any key to continue...");
        cv::waitKey(0);
      }
    }
  }

  cv::destroyAllWindows();

  return true;
}

bool CamDataLoader::SaveDetections(const std::string& output_path) {
  if (!sequence_ || sequence_->Size() == 0) {
    spdlog::warn("No camera data to save. Please load data first.");
    return false;
  }

  spdlog::info("Saving camera data to: {}", output_path);

  try {
    std::ofstream output_file(output_path);
    if (!output_file.is_open()) {
      spdlog::error("Failed to open output JSON file: {}", output_path);
      return false;
    }

    nlohmann::json nlm_json;
    nlm_json["img_width"] = img_width_;
    nlm_json["img_height"] = img_height_;
    nlm_json["sequence"] = *sequence_;

    // Serialize.
    output_file << nlm_json.dump(2);
    output_file.close();

  } catch (const std::exception& e) {
    spdlog::error("JSON writing error: {}", e.what());
    return false;
  } catch (...) {
    spdlog::error("Unknown error writing JSON.");
    return false;
  }

  return true;
}

bool CamDataLoader::ReadDetections(const std::string& input_path) {
  spdlog::info("Reading camera data from: {}", input_path);

  try {
    std::ifstream input_file(input_path);
    if (!input_file.is_open()) {
      spdlog::error("Failed to open input JSON file: {}", input_path);
      return false;
    }

    nlohmann::json nlm_json;
    input_file >> nlm_json;
    input_file.close();

    // Deserialize image dimensions (required fields).
    if (!nlm_json.contains("img_width")) {
      spdlog::error("Missing required field 'img_width' in JSON file.");
      return false;
    }
    if (!nlm_json.contains("img_height")) {
      spdlog::error("Missing required field 'img_height' in JSON file.");
      return false;
    }
    if (!nlm_json.contains("sequence")) {
      spdlog::error("Missing required field 'sequence' in JSON file.");
      return false;
    }

    img_width_ = nlm_json["img_width"].get<int>();
    img_height_ = nlm_json["img_height"].get<int>();

    // Deserialize sequence.
    sequence_ = CamSequence::Create();
    nlm_json["sequence"].get_to(*sequence_);

  } catch (const std::exception& e) {
    spdlog::error("JSON parsing error: {}", e.what());
    return false;
  } catch (...) {
    spdlog::error("Unknown error reading JSON.");
    return false;
  }

  return true;
}

}  // namespace xr_ucalib

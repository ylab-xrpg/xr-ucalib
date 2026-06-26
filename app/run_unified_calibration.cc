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

#include <spdlog/spdlog.h>

#include <cstdlib>
#include <filesystem>

#include "xr_ucalib/uc_cam_calib/cam_rig_calib/cam_rig_calibrator.h"
#include "xr_ucalib/uc_cam_calib/sfm_calib/sfm_calibrator.h"
#include "xr_ucalib/uc_common/utils/timer.h"
#include "xr_ucalib/uc_unified_calib/calibrator/unified_calibrator.h"

int main(int argc, char** argv) {
  spdlog::set_level(spdlog::level::info);
  spdlog::set_pattern("%^[%l]%$ %v");

  // Step 1: Set the system working directory.
  if (argc != 2 && argc != 5) {
    // clang-format off
    spdlog::error(
        "Invalid input parameters.\n\n"
        "Usage:\n"
        "  {} <work_directory>\n"
        "  {} <config_path> <sensor_data_dir> <workspace_dir> <output_path>\n\n"
        "Description:\n"
        "  If a single <work_directory> is provided, the program assumes default file organization:\n"
        "    - Config path:      <work_directory>/input_config.json\n"
        "    - Sensor data dir:  <work_directory>/sensor_data\n"
        "    - Workspace dir:    <work_directory>/ucalib_ws\n"
        "    - Output path:      <work_directory>/output_calib_params.json\n\n"
        "  Alternatively, you can provide all paths explicitly.\n"
        "  <config_path>      Path to the calibration configuration JSON file.\n"
        "  <sensor_data_dir>  Directory containing the sensor data.\n"
        "  <workspace_dir>    Directory to store intermediate workspace data.\n"
        "  <output_path>      Path to save the calibration results.\n\n"
        "Examples:\n"
        "  {} ../data/test_data_handheld\n"
        "  {} ../data/test_data_handheld/input_config.json ../data/test_data_handheld/sensor_data "
        "../data/test_data_handheld/ucalib_ws ../data/test_data_handheld/output_calib_params.json\n",
        argv[0], argv[0], argv[0], argv[0]);
    // clang-format on

    std::exit(EXIT_FAILURE);
  }

  std::string config_path;
  std::string sensor_data_dir;
  std::string workspace_dir;
  std::string output_path;

  if (argc == 2) {
    std::filesystem::path work_dir(argv[1]);
    config_path = (work_dir / "input_config.json").string();
    sensor_data_dir = (work_dir / "sensor_data").string();
    workspace_dir = (work_dir / "ucalib_ws").string();
    output_path = (work_dir / "output_calib_params.json").string();
  } else {
    config_path = argv[1];
    sensor_data_dir = argv[2];
    workspace_dir = argv[3];
    output_path = argv[4];
  }

  // ===========================================================================

  // Timers for each module.
  xr_ucalib::Timer total_timer;
  xr_ucalib::Timer module_timer;
  total_timer.Start();

  // Step 2: Set up our calibration system.
  spdlog::info("=======================================================");
  spdlog::info("============== Set Up Calibration System ==============");
  spdlog::info("=======================================================");
  module_timer.Restart();

  // Step 2.1: Load system configuration.
  auto system_config = xr_ucalib::SystemConfig::Create();
  if (!system_config->FromJson(config_path)) {
    spdlog::error("Failed to read system configuration.");
    std::exit(EXIT_FAILURE);
  }
  system_config->sensor_data_dir = sensor_data_dir;
  system_config->workspace_dir = workspace_dir;

  bool enable_ba_refine = system_config->cam_calib_config.enable_ba_refine;
  bool enable_unified_calib =
      system_config->unified_calib_config.enable_unified_calib;

  // Step 2.2: Load sensor data.
  auto sensor_manager = xr_ucalib::SensorManager::Create();
  if (!sensor_manager->LoadSensorData(system_config)) {
    spdlog::error("Failed to load sensor data.");
    std::exit(EXIT_FAILURE);
  }

  // Step 2.3: Set up default calibration parameters.
  auto calib_parameters = xr_ucalib::CalibParameters::Create();
  if (!calib_parameters->SetUpDefaultValues(sensor_manager,
                                            enable_unified_calib)) {
    spdlog::error("Failed to set up default calibration parameters.");
    std::exit(EXIT_FAILURE);
  }

  spdlog::info("System setup completed in {:.3f} s.", module_timer.ElapsedSeconds());

  // ===========================================================================

  // Step 3: Run camera-only calibration first.
  spdlog::info("=======================================================");
  spdlog::info("============= Run Camera-only Calibration =============");
  spdlog::info("=======================================================");

  // Step 3.1: Perform SFM-based camera calibration.
  module_timer.Restart();
  auto sfm_calibrator = xr_ucalib::SfmCalibrator::Create(
      system_config, sensor_manager, calib_parameters);
  if (!sfm_calibrator->RunCalibration()) {
    spdlog::error("SFM-based camera calibration failed.");
    std::exit(EXIT_FAILURE);
  }
  spdlog::info("SFM calibration completed in {:.3f} s.",
               module_timer.ElapsedSeconds());

  // Step 3.2: Perform camera rig calibration (bundle adjustment refinement) if
  // enabled.
  if (enable_ba_refine) {
    module_timer.Restart();
    auto cam_rig_calibrator = xr_ucalib::CamRigCalibrator::Create(
        system_config, sensor_manager, calib_parameters);
    if (!cam_rig_calibrator->RunCalibration()) {
      spdlog::error("Camera rig calibration failed.");
      std::exit(EXIT_FAILURE);
    }
    spdlog::info("Camera rig calibration (BA) completed in {:.3f} s.",
                 module_timer.ElapsedSeconds());
  }

  if (!enable_unified_calib) {
    calib_parameters->ToJson(output_path);
    spdlog::info("Total calibration pipeline completed in {:.3f} s ({:.2f} min).",
                 total_timer.ElapsedSeconds(), total_timer.ElapsedMinutes());
    return 0;
  }

  // ===========================================================================

  // Step 4: Run unified multi-sensor calibration if enabled.

  spdlog::info("=======================================================");
  spdlog::info("========= Run Unified Multi-Sensor Calibration ========");
  spdlog::info("=======================================================");

  module_timer.Restart();
  auto unified_calibrator = xr_ucalib::UnifiedCalibrator::Create(
      system_config, sensor_manager, calib_parameters);

  if (!unified_calibrator->RunCalibration()) {
    spdlog::error("Unified multi-sensor calibration failed.");
    std::exit(EXIT_FAILURE);
  }
  spdlog::info("Unified calibration completed in {:.3f} s.",
               module_timer.ElapsedSeconds());

  calib_parameters->ToJson(output_path);

  // ===========================================================================

  spdlog::info("Total calibration pipeline completed in {:.3f} s ({:.2f} min).",
               total_timer.ElapsedSeconds(), total_timer.ElapsedMinutes());

  return 0;
}

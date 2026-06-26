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
#include "xr_ucalib/uc_cam_calib/sfm_calib/sfm_calibrator.h"

#include <atomic>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Eigen>

#include <colmap/util/misc.h>
#include <colmap/feature/types.h>
#include <colmap/controllers/feature_extraction.h>
#include <colmap/controllers/feature_matching.h>
#include <colmap/controllers/incremental_mapper.h>
#include <colmap/controllers/option_manager.h>
#include <colmap/controllers/bundle_adjustment.h>
#include <colmap/scene/reconstruction.h>
#include <spdlog/spdlog.h>

#include "xr_ucalib/uc_cam_calib/sfm_calib/handeye_calibrator.h"
#include "xr_ucalib/uc_common/utils/console_silencer.h"
// clang-format on

namespace xr_ucalib {

bool SfmCalibrator::RunCalibration() {
  spdlog::info("------------------- SFM Calibration -------------------");

  if (system_config_->workspace_dir == "") {
    spdlog::error(
        "Workspace directory is not set in the system configuration.");
    return false;
  }
  std::string work_dir = system_config_->workspace_dir + "/sfm_calib";

  if (!(calib_parameters_->param_status &
        CalibParameters::ParamStatus::SETUP)) {
    spdlog::error(
        "Calibration parameters must be set up before SFM calibration. Current "
        "param_status: {}",
        static_cast<int>(calib_parameters_->param_status));
    return false;
  }

  // ===========================================================================

  // Step 1: Perform SFM for each camera independently.
  auto& cam_sequences = sensor_manager_->GetAllCamSequences();

  // Map for storing reconstruction results for each camera.
  ReconMap reconstructions;
  // Map for storing image maps for each camera.
  std::map<std::string, ImageMap> img_maps;

  // Iterate over each camera sequence.
  for (auto const& [label, cam_seq] : cam_sequences) {
    spdlog::info(" - Starting SFM calibration for camera: {}.", label);

    ReconstructionPtr reconstruction;
    // Map from COLMAP image IDs to camera frames.
    ImageMap img_map;

    std::string curr_work_dir = work_dir + "/" + label;
    std::string db_path = curr_work_dir + "/" + label + "_database.db";
    if (std::filesystem::exists(curr_work_dir)) {
      std::filesystem::remove_all(curr_work_dir);
    }
    std::filesystem::create_directories(curr_work_dir);

    // Step 1.1: Construct COLMAP database for the camera.
    if (!ConstructDatabase(label, cam_seq, db_path, img_map)) {
      spdlog::error("Failed to construct database for: {}.", label);
      return false;
    }

    // Step 1.2: Run COLMAP mapper to perform SFM.
    if (!RunMapper(db_path, img_map, curr_work_dir, reconstruction)) {
      spdlog::error("Failed to run mapper for camera: {}.", label);
      return false;
    }

    spdlog::info("SFM for {} completed, #Images: {}, #3D Points: {}.", label,
                 reconstruction->NumImages(), reconstruction->NumPoints3D());

    // Store reconstruction result.
    reconstructions[label] = reconstruction;
    img_maps[label] = img_map;
  }

  // ===========================================================================

  // Step 2: Convert and scale the reconstruction results to real-world units.
  ReconGeometryMap recon_geometry_map;
  // Iterate over each COLMAP reconstruction result.
  for (const auto& [label, recon] : reconstructions) {
    ReconstructionGeometry::Ptr recon_geometry;
    if (!ConvertAndScaleReconstruction(recon, label, img_maps[label],
                                       recon_geometry)) {
      spdlog::error(
          "Failed to convert and scale reconstruction for camera: {}.", label);
      return false;
    }

    recon_geometry_map[label] = recon_geometry;
  }

  // ===========================================================================

  // Step 3: Perform hand-eye calibration and Umeyama alignment between
  // reconstructions.
  // Transformation from i-th camera body frame to base camera body frame.
  std::map<std::string, Eigen::Matrix4d> T_Cb_Ci_map;
  // Transformation from i-th camera world frame to base camera world frame.
  std::map<std::string, Eigen::Matrix4d> T_Wb_Wi_map;

  std::string base_camera_label = calib_parameters_->base_camera_label;
  auto base_camera_pose =
      recon_geometry_map.at(base_camera_label)->camera_pose_W_C;
  for (auto& [label, recon_geometry] : recon_geometry_map) {
    if (label == base_camera_label) continue;

    // Solve extrinsics between camera bodies.
    Eigen::Matrix4d T_Cb_Ci = Eigen::Matrix4d::Identity();
    auto current_camera_pose = recon_geometry->camera_pose_W_C;
    if (!HandEyeCalibrator::SolveBodyExtrinsics(base_camera_pose,
                                                current_camera_pose, T_Cb_Ci)) {
      spdlog::error("Failed to solve body extrinsics between {} and {}.",
                    base_camera_label, label);
      return false;
    }

    T_Cb_Ci_map[label] = T_Cb_Ci;
    recon_geometry->TransformBody(T_Cb_Ci.inverse());

    // Align world frames.
    Eigen::Matrix4d T_Wb_Wi = Eigen::Matrix4d::Identity();
    current_camera_pose = recon_geometry->camera_pose_W_C;
    if (!HandEyeCalibrator::AlignWorldFrames(base_camera_pose,
                                             current_camera_pose, T_Wb_Wi)) {
      spdlog::error("Failed to align world frames between {} and {}.",
                    base_camera_label, label);
      return false;
    }

    T_Wb_Wi_map[label] = T_Wb_Wi;
    recon_geometry->TransformWorld(T_Wb_Wi);
  }

  // ===========================================================================

  // Step 4: Fuse all reconstructions and compute relative target poses.
  std::map<int, Eigen::Vector3d> fused_points3d;
  std::map<int, Eigen::Matrix4d> T_T0_Ti_map;

  if (!FuseReconstructionPoints(recon_geometry_map, fused_points3d)) {
    spdlog::error("Failed to fuse reconstruction points from all cameras.");
    return false;
  }

  if (!ComputeRelativeTargetPoses(fused_points3d, T_T0_Ti_map)) {
    spdlog::error("Failed to compute relative target poses.");
    return false;
  }

  // ===========================================================================

  // Step 5: Update calibration parameters with SFM results and print them.
  // Update camera extrinsics.
  for (auto& [label, T_Cb_Ci] : T_Cb_Ci_map) {
    calib_parameters_->trans_Cb_Ci[label] = T_Cb_Ci.block<3, 1>(0, 3);
    calib_parameters_->rot_Cb_Ci[label] =
        Sophus::SO3d(T_Cb_Ci.block<3, 3>(0, 0));
  }
  // Update target extrinsics. The first target is assumed to be at the system
  // world.
  for (auto& [target_idx, T_T0_Ti] : T_T0_Ti_map) {
    calib_parameters_->trans_W_Ti[target_idx] = T_T0_Ti.block<3, 1>(0, 3);
    calib_parameters_->rot_W_Ti[target_idx] =
        Sophus::SO3d(T_T0_Ti.block<3, 3>(0, 0));
  }
  // Update camera intrinsics.
  for (auto& [label, reconstruction] : reconstructions) {
    auto cam_intrinsic = calib_parameters_->cam_intrinsics.at(label);
    cam_intrinsic->parameters =
        reconstruction->Cameras().begin()->second.Params();
  }

  spdlog::info("SFM calibration completed.");

  spdlog::info("--------------- SFM Calibration Results ---------------");

  // Print the results.
  PrintCalibrationResults();

  calib_parameters_->param_status |= CalibParameters::ParamStatus::SFM_CALIB;

  // ===========================================================================

  return true;
}

bool SfmCalibrator::ConstructDatabase(const std::string& label,
                                      const CamSequence::Ptr& cam_seq,
                                      const std::string& db_path,
                                      ImageMap& img_map) {
  // Step 1: Down-sample camera sequence based on configuration.
  const int kDownSampleRate =
      system_config_->cam_calib_config.cam_down_sample_rate;
  constexpr int kMinKeypoints = 20;

  auto down_sampled_cam_seq = CamSequence::Create();
  int ignored_frames = 0;
  double raw_num = static_cast<double>(cam_seq->Size()) / kDownSampleRate;
  for (size_t i = 0; i < cam_seq->Size(); i += kDownSampleRate) {
    const auto& cam_frame = cam_seq->At(i);
    if (cam_frame->keypoints.size() < kMinKeypoints) {
      ignored_frames++;
      continue;
    }
    down_sampled_cam_seq->Add(cam_frame);
  }

  if (static_cast<double>(ignored_frames) > raw_num / 10.) {
    spdlog::warn(
        "More than 10% of frames are skipped due to insufficient keypoints "
        "(minimum required: {}). Please ensure good visibility of the fiducial "
        "target in current camera.",
        kMinKeypoints);
  }

  spdlog::info("Original frames: {}, Down-sampled frames: {}.", cam_seq->Size(),
               down_sampled_cam_seq->Size());

  if (down_sampled_cam_seq->Size() < 10) {
    spdlog::error("Too few frames ({}) left after down-sampling and filtering",
                  down_sampled_cam_seq->Size());
    return false;
  }

  // ===========================================================================

  // Step 2: Construct COLMAP database and add camera to it.
  auto database = std::make_unique<colmap::Database>(db_path);

  auto cam_intrinsic = calib_parameters_->cam_intrinsics.at(label);
  colmap::Camera colmap_cam;
  if (!InitialColmapCamera(cam_intrinsic, colmap_cam)) {
    spdlog::error("Failed to initialize COLMAP camera");
    return false;
  }

  if (!colmap_cam.VerifyParams()) {
    spdlog::error("COLMAP camera parameters verification failed.");
    return false;
  }

  colmap::camera_t colmap_cam_id = database->WriteCamera(colmap_cam);
  cam_intrinsic->colmap_cam_id = colmap_cam_id;

  // ===========================================================================

  // Step 3: Add images and keypoints to database.
  for (auto const& cam_frame : *down_sampled_cam_seq) {
    colmap::Image colmap_img;
    colmap_img.SetCameraId(colmap_cam_id);
    colmap_img.SetName(cam_frame->img_path);

    colmap::image_t img_id = database->WriteImage(colmap_img);
    img_map[img_id] = cam_frame;

    colmap::FeatureKeypoints colmap_kps;
    // Convert keypoints to COLMAP format. The COLMAP keypoint index
    // corresponds to the order as which they appear in the CamFrame's
    // keypoints map.
    for (const auto& [landmark_id, kp] : cam_frame->keypoints) {
      colmap_kps.push_back(colmap::FeatureKeypoint(kp.x(), kp.y()));
    }
    database->WriteKeypoints(img_id, colmap_kps);
  }

  // ===========================================================================

  // Step 4: Match keypoints between image pairs based on landmark IDs, and
  // write to database.
  auto colmap_imgs = database->ReadAllImages();
  if (colmap_imgs.size() != img_map.size()) {
    // This should never happen.
    spdlog::error(
        "Mismatch between number of images in database and image map: {} vs "
        "{}.",
        colmap_imgs.size(), img_map.size());
    return false;
  }

  // Step 4.1: Determine adaptive matching strategy based on image count.
  // Match all pairs if image count is small, use sparse matching for large
  // datasets to balance efficiency and completeness.
  size_t num_images = colmap_imgs.size();
  int adjacent_match_range = 0;
  int skip_match_interval = 0;

  if (num_images < 50) {
    // Very few images: match all pairs
    adjacent_match_range = num_images;  // Match all
    skip_match_interval = 1;            // No skipping
  } else if (num_images < 100) {
    // Moderate image count: match adjacent frames + some sparse pairs
    adjacent_match_range = 10;
    skip_match_interval = 5;
  } else {
    // Large image count: aggressive sparse matching
    adjacent_match_range = 10;
    skip_match_interval = 10;
  }

  // Pre-compute matching pairs.
  std::vector<std::pair<size_t, size_t>> match_pairs;
  for (size_t i = 0; i < num_images; ++i) {
    for (size_t j = i + 1; j < num_images; ++j) {
      size_t gap = j - i;
      // Match if within adjacent range or at skip interval.
      if (gap <= static_cast<size_t>(adjacent_match_range) ||
          (gap % skip_match_interval == 0)) {
        match_pairs.push_back({i, j});
      }
    }
  }

  // ===========================================================================

  // Step 4.2: Prepare worker function for multi-threaded matching.
  std::mutex db_mutex;  // database write mutex
  std::mutex io_mutex;  // console IO mutex
  std::atomic<int> current_processed(0);
  std::atomic<size_t> failed_pairs(0);
  const int kMultiThreadNum =
      std::max(1, system_config_->cam_calib_config.multi_thread_num);
  const int kMinTvgInliers = 15;

  auto worker_func = [&](int start_idx, int end_idx) {
    for (int p = start_idx; p < end_idx; ++p) {
      size_t i = match_pairs[p].first;
      size_t j = match_pairs[p].second;

      CamFrame::Ptr cam_frame1 = img_map.at(colmap_imgs[i].ImageId());
      CamFrame::Ptr cam_frame2 = img_map.at(colmap_imgs[j].ImageId());

      // Match keypoints based on landmark IDs. The matching information
      // corresponds to the indices of the matched keypoints in the keypoint
      // containers of the current frame.
      colmap::FeatureMatches matches = MatchTwoFrames(cam_frame1, cam_frame2);

      // Estimate two-view geometry and filter matches using RANSAC.
      colmap::TwoViewGeometryOptions tvg_options;
      tvg_options.ransac_options.min_num_trials = 10;
      tvg_options.ransac_options.max_num_trials = 100;

      std::vector<Eigen::Vector2d> points1, points2;
      points1.reserve(cam_frame1->keypoints.size());
      points2.reserve(cam_frame2->keypoints.size());
      for (const auto& [landmark_id, kp] : cam_frame1->keypoints) {
        points1.push_back(kp);
      }
      for (const auto& [landmark_id, kp] : cam_frame2->keypoints) {
        points2.push_back(kp);
      }

      auto tvg_result = colmap::EstimateCalibratedTwoViewGeometry(
          colmap_cam, points1, colmap_cam, points2, matches, tvg_options);

      if (tvg_result.inlier_matches.size() < kMinTvgInliers) {
        failed_pairs++;
      } else {
        // Write matches and two-view geometry to database.
        std::lock_guard<std::mutex> lock(db_mutex);
        database->WriteMatches(colmap_imgs[i].ImageId(),
                               colmap_imgs[j].ImageId(), matches);
        database->WriteTwoViewGeometry(colmap_imgs[i].ImageId(),
                                       colmap_imgs[j].ImageId(), tvg_result);
      }

      int processed = ++current_processed;
      if (processed % 10 == 0 ||
          processed == static_cast<int>(match_pairs.size())) {
        std::lock_guard<std::mutex> lock(io_mutex);
        std::printf("\rMatching keypoints for COLMAP mapping: %d/%zu",
                    processed, match_pairs.size());
        std::fflush(stdout);
      }
    }
  };

  // ===========================================================================

  // Step 4.3: Launch multi-threaded matching.
  std::vector<std::thread> thread_pool;
  size_t total_tasks = match_pairs.size();
  int task_step = (total_tasks + kMultiThreadNum - 1) / kMultiThreadNum;
  for (int i = 0; i < kMultiThreadNum; ++i) {
    int start = i * task_step;
    int end = std::min(start + task_step, static_cast<int>(total_tasks));
    if (start < end) {
      thread_pool.emplace_back(worker_func, start, end);
    }
  }

  for (auto& thread : thread_pool) {
    thread.join();
  }
  std::printf("\n");

  size_t successful_pairs = match_pairs.size() - failed_pairs.load();
  if (failed_pairs.load() * 2 > successful_pairs) {
    spdlog::warn(
        "Too many failed image pairs in keypoint matching (failed: {} vs "
        "successful: {}). SFM calibration may fail.",
        failed_pairs.load(), successful_pairs);
  }

  // ===========================================================================

  database->Close();

  return true;
}

bool SfmCalibrator::RunMapper(const std::string& db_path,
                              const ImageMap& img_map,
                              const std::string& work_dir,
                              ReconstructionPtr& reconstruction) {
  reconstruction = std::make_shared<colmap::Reconstruction>();

  // Configure COLMAP options for mapping.
  colmap::OptionManager options;
  options.AddDatabaseOptions();
  options.AddImageOptions();
  options.AddMapperOptions();
  options.AddBundleAdjustmentOptions();

  *options.database_path = db_path;
  *options.image_path = "";

  options.mapper->extract_colors = false;
  options.mapper->ba_refine_principal_point = true;
  options.mapper->ba_refine_extra_params = true;
  options.mapper->ba_refine_focal_length = true;
  options.bundle_adjustment->solver_options.sparse_linear_algebra_library_type =
      ceres::SUITE_SPARSE;

  // Reconstruction manager to store mapping results.
  auto reconstruction_manager =
      std::make_shared<colmap::ReconstructionManager>();

  // Core mapping controller.
  colmap::IncrementalMapperController mapper(
      options.mapper, *options.image_path, *options.database_path,
      reconstruction_manager);

  // Add callback to save intermediate reconstruction results.
  size_t recon_num = 0;
  mapper.AddCallback(
      colmap::IncrementalMapperController::LAST_IMAGE_REG_CALLBACK, [&]() {
        if (reconstruction_manager->Size() > recon_num) {
          const std::string reconstruction_path = colmap::JoinPaths(
              work_dir, "reconstruction" + std::to_string(recon_num));

          colmap::CreateDirIfNotExists(reconstruction_path);
          reconstruction_manager->Get(recon_num)->WriteText(
              reconstruction_path);

          options.Write(colmap::JoinPaths(reconstruction_path, "project.ini"));

          recon_num = reconstruction_manager->Size();
        }
      });

  // Start mapping process.
  {
    spdlog::info("Running COLMAP incremental mapper, it may take a while...");
    ConsoleSilencer silencer;  // Silence COLMAP console output.
    mapper.Start();
    mapper.Wait();
  }

  if (reconstruction_manager->Size() == 0) {
    spdlog::error("No reconstruction is generated by COLMAP mapper. ");
    return false;
  }

  // Find and save the largest reconstruction.
  size_t largest_recon_index = 0;
  size_t largest_reg_img_num = 0;

  for (size_t i = 0; i < reconstruction_manager->Size(); i++) {
    auto recon = reconstruction_manager->Get(i);
    size_t reg_img_num = recon->RegImageIds().size();

    if (reg_img_num > largest_reg_img_num) {
      largest_reg_img_num = reg_img_num;
      largest_recon_index = i;
    }
  }

  if (largest_reg_img_num == 0) {
    spdlog::error("No images are registered in the largest reconstruction.");
    return false;
  }

  if (largest_reg_img_num < img_map.size() / 2) {
    spdlog::warn(
        "Only {} / {} images are registered in the largest reconstruction. "
        "Ensure sufficient co-visibility between images.",
        largest_reg_img_num, img_map.size());
  }

  reconstruction = reconstruction_manager->Get(largest_recon_index);

  return true;
}

bool SfmCalibrator::InitialColmapCamera(
    const CamIntrinsicBase::Ptr& cam_intrinsic, colmap::Camera& colmap_cam) {
  // Set initial camera intrinsic parameter with  default values.
  // TODO: Use better initial values via camera initialization algorithms.
  cam_intrinsic->parameters = std::vector<double>(
      {static_cast<double>(cam_intrinsic->initial_focal_length),
       static_cast<double>(cam_intrinsic->initial_focal_length),
       static_cast<double>(cam_intrinsic->width) / 2.0,
       static_cast<double>(cam_intrinsic->height) / 2.0, 0., 0., 0., 0.});

  // Set COLMAP camera model and parameters.
  if (cam_intrinsic->cam_model_type == CamModelType::RADTAN) {
    colmap_cam.SetModelIdFromName("OPENCV");
  } else if (cam_intrinsic->cam_model_type == CamModelType::EQUIDISTANT) {
    colmap_cam.SetModelIdFromName("OPENCV_FISHEYE");
  } else {
    spdlog::error("Unsupported camera model type for COLMAP initialization.");
    return false;
  }

  colmap_cam.SetWidth(cam_intrinsic->width);
  colmap_cam.SetHeight(cam_intrinsic->height);

  colmap_cam.Params().clear();
  colmap_cam.Params() = cam_intrinsic->parameters;

  colmap_cam.SetPriorFocalLength(false);

  return true;
}

colmap::FeatureMatches SfmCalibrator::MatchTwoFrames(
    const CamFrame::Ptr& cam_frame1, const CamFrame::Ptr& cam_frame2) {
  colmap::FeatureMatches matches;
  for (const auto& [landmark_id1, kp1] : cam_frame1->keypoints) {
    for (const auto& [landmark_id2, kp2] : cam_frame2->keypoints) {
      // Match based on landmark IDs.
      if (landmark_id1 == landmark_id2) {
        colmap::FeatureMatch match;
        // Get indices of the matched keypoints in the current frames.
        match.point2D_idx1 = static_cast<colmap::point2D_t>(
            std::distance(cam_frame1->keypoints.begin(),
                          cam_frame1->keypoints.find(landmark_id1)));
        match.point2D_idx2 = static_cast<colmap::point2D_t>(
            std::distance(cam_frame2->keypoints.begin(),
                          cam_frame2->keypoints.find(landmark_id2)));
        matches.push_back(match);
        break;
      }
    }
  }

  return matches;
}

bool SfmCalibrator::ConvertAndScaleReconstruction(
    const ReconstructionPtr& reconstruction, const std::string& label,
    const ImageMap& img_map, ReconstructionGeometry::Ptr& recon_geometry) {
  recon_geometry = ReconstructionGeometry::Create();
  recon_geometry->label = label;

  // Pre-compute a mapping from COLMAP point2D index to our landmark ID for
  // each image. The COLMAP point2D index corresponds to the order as which
  // they appear in the CamFrame's keypoints map. And our landmark ID is the
  // unique identifier corresponding to calibration target corners.
  std::map<colmap::image_t, std::vector<int>> image_id_to_landmark_ids;
  for (const auto& [img_id, cam_frame] : img_map) {
    auto& landmark_ids = image_id_to_landmark_ids[img_id];
    landmark_ids.reserve(cam_frame->keypoints.size());
    for (const auto& [landmark_id, _] : cam_frame->keypoints) {
      landmark_ids.push_back(landmark_id);
    }
  }

  // ===========================================================================

  // Step 1: Convert 3D points in COLMAP reconstruction to our format.
  // Handle the case where one landmark ID is split into multiple 3D points by
  // keeping the one with the longest track.
  std::map<int, std::vector<std::pair<size_t, Eigen::Vector3d>>>
      landmark_candidates;

  // Iterate through all 3D points in the reconstruction.
  for (const auto& recon_point : reconstruction->Points3D()) {
    // Gather all landmark IDs observing the current 3D point.
    std::vector<int> keypoint_ids;
    const auto& track_elements = recon_point.second.Track().Elements();
    keypoint_ids.reserve(track_elements.size());

    for (const auto& track : track_elements) {
      auto iter = image_id_to_landmark_ids.find(track.image_id);
      if (iter == image_id_to_landmark_ids.end()) {
        spdlog::error(
            "Image ID {} not found in image map during reconstruction 3D "
            "points conversion.",
            track.image_id);
        return false;
      }

      const auto& landmark_ids = iter->second;
      if (track.point2D_idx >= landmark_ids.size()) {
        spdlog::error(
            "Keypoint index {} out of range for image {} during "
            "reconstruction conversion.",
            track.point2D_idx, track.image_id);
        return false;
      }
      // Get the landmark ID corresponding to the current track element.
      keypoint_ids.push_back(landmark_ids[track.point2D_idx]);
    }

    if (keypoint_ids.empty()) continue;

    // All landmark IDs observing the same 3D point should be the same.
    bool all_same =
        std::all_of(keypoint_ids.begin(), keypoint_ids.end(),
                    [&](int v) { return v == keypoint_ids.front(); });

    if (!all_same) {
      spdlog::error(
          "Inconsistent landmark IDs for 3D point {} during reconstruction "
          "conversion.",
          recon_point.first);

      return false;
    }

    // Add the current 3D point as a candidate for the corresponding landmark
    // ID.
    int landmark_id = keypoint_ids.front();
    size_t track_length = recon_point.second.Track().Length();
    landmark_candidates[landmark_id].push_back(
        {track_length, recon_point.second.XYZ()});
  }

  // Select the best 3D point for each landmark ID.
  for (const auto& [id, candidates] : landmark_candidates) {
    if (candidates.empty()) continue;

    if (candidates.size() > 1) {
      spdlog::debug(
          "Landmark {} is split into {} 3D points in reconstruction. Keeping "
          "the one with the longest track.",
          id, candidates.size());
    }

    // Find the candidate with the maximum track length.
    const auto& best_candidate =
        *std::max_element(candidates.begin(), candidates.end(),
                          [](const std::pair<size_t, Eigen::Vector3d>& a,
                             const std::pair<size_t, Eigen::Vector3d>& b) {
                            return a.first < b.first;
                          });

    recon_geometry->points3d[id] = best_candidate.second;
  }

  // ===========================================================================

  // Step 2： Convert camera poses in COLMAP reconstruction to our format.
  for (const auto& recon_img : reconstruction->Images()) {
    StampedPose cam_pose;
    // Get timestamp from image map.
    auto frame_iter = img_map.find(recon_img.second.ImageId());
    if (frame_iter == img_map.end()) {
      spdlog::error(
          "Image ID {} not found in image map during reconstruction camera "
          "conversion.",
          recon_img.second.ImageId());

      return false;
    }
    const auto& cam_frame = frame_iter->second;
    cam_pose.timestamp = cam_frame->timestamp;

    // Convert COLMAP camera pose (CameraFromWorld) to our format.
    Eigen::Matrix<double, 3, 4> T_W_C =
        colmap::Inverse(recon_img.second.CamFromWorld()).ToMatrix();

    cam_pose.trans = T_W_C.block<3, 1>(0, 3);
    cam_pose.rot_q = Eigen::Quaterniond(T_W_C.block<3, 3>(0, 0)).normalized();

    recon_geometry->camera_pose_W_C.push_back(cam_pose);
  }

  // ===========================================================================

  // Step 3: Compare the reconstructed fiducial edge lengths with the actual
  // sizes to estimate scale factor.
  auto target_corners = sensor_manager_->GetTargetCorners();
  if (!target_corners) {
    spdlog::error("Target corners not available for scale estimation.");
    return false;
  }

  // Use a map to store edge lengths for each target (key: target_idx).
  std::map<int, std::vector<double>> edge_lengths;

  // Step 3.1: Iterate through reconstructed 3D points to compute fiducial
  // edge lengths.
  for (auto it = recon_geometry->points3d.begin();
       it != recon_geometry->points3d.end(); ++it) {
    auto next_it = std::next(it);
    if (next_it == recon_geometry->points3d.end()) break;

    int id_1 = it->first;
    int id_2 = next_it->first;

    // Check if IDs are sequential.
    if (id_2 != id_1 + 1) continue;

    // Find which target these points belong to.
    int target_idx = target_corners->GetTargetIdx(id_1);

    if (target_idx == -1) continue;

    const auto& range = target_corners->target_id_ranges.at(target_idx);

    // Check if the second point is also in the same target.
    if (id_2 > range.second) continue;

    // Check if it is a valid edge within a tag (0-1, 1-2, 2-3)
    // The edge (3-4) is a jump between tags
    int start_id = range.first;
    int local_id = id_1 - start_id;
    if (local_id % 4 == 3) continue;

    double dist = (it->second - next_it->second).norm();
    edge_lengths[target_idx].push_back(dist);
  }

  // Step 3.2: Estimate scale for each target and check consistency.
  std::vector<double> valid_scales;
  constexpr double kMaxStdDevRatio = 0.05;  // 5%

  const auto& target_configs = sensor_manager_->GetAllTargetConfigs();
  for (const auto& [target_idx, target_config] : target_configs) {
    const auto& lengths = edge_lengths[target_idx];
    int total_points =
        target_config.fiducial_rows * target_config.fiducial_cols * 4;
    const size_t kMinEdgesThreshold =
        std::min<size_t>(20, static_cast<size_t>(total_points) / 4);

    // If too few edges are found, skip this target.
    if (lengths.size() < kMinEdgesThreshold) {
      spdlog::info(
          "Camera {}: Insufficient edges for target {} ({} < {}) to estimate "
          "scale. Skipping.",
          label, target_idx, lengths.size(), kMinEdgesThreshold);
    }

    double sum = 0.0;
    for (double len : lengths) sum += len;
    double mean = sum / lengths.size();

    double sq_sum = 0.0;
    for (double len : lengths) sq_sum += (len - mean) * (len - mean);
    double std_dev = std::sqrt(sq_sum / lengths.size());

    // Check standard deviation (in unscaled units, compared to mean)
    if (std_dev > mean * kMaxStdDevRatio) {
      spdlog::warn(
          "Camera {}: Large standard deviation for target {} edges (std/mean "
          "= {:.3f} > {:.3f}). Reconstruction might be noisy.",
          label, target_idx, std_dev / mean, kMaxStdDevRatio);
    }

    double scale = target_config.fiducial_size / mean;
    valid_scales.push_back(scale);
  }

  if (valid_scales.empty()) {
    spdlog::error("Camera {}: Failed to estimate scale from any target.",
                  label);
    return false;
  }

  // ===========================================================================

  // Step 4: Analyze scale consistency across targets and apply final scale.
  // Check consistency
  constexpr double kMaxScaleDiffRatio = 0.02;  // 2%
  if (valid_scales.size() > 1) {
    double max_diff_ratio = 0.0;
    for (size_t i = 0; i < valid_scales.size(); ++i) {
      for (size_t j = i + 1; j < valid_scales.size(); ++j) {
        double diff = std::abs(valid_scales[i] - valid_scales[j]);
        double ratio = diff / std::max(valid_scales[i], valid_scales[j]);
        if (ratio > max_diff_ratio) max_diff_ratio = ratio;
      }
    }

    if (max_diff_ratio > kMaxScaleDiffRatio) {
      spdlog::warn(
          "Camera {}: Inconsistent scales between targets (max relative diff "
          "= {:.3f} > {:.3f}).",
          label, max_diff_ratio, kMaxScaleDiffRatio);
    }
  }

  // Use the average of valid scales
  double final_scale = 0.0;
  for (double s : valid_scales) final_scale += s;
  final_scale /= valid_scales.size();

  recon_geometry->Scale(final_scale);

  // ===========================================================================

  return true;
}

bool SfmCalibrator::FuseReconstructionPoints(
    const ReconGeometryMap& recon_geometry_map,
    std::map<int, Eigen::Vector3d>& fused_points3d) {
  if (recon_geometry_map.empty()) {
    spdlog::error("No reconstruction geometries provided for fusion.");
    return false;
  }

  // If only one reconstruction, no need to fuse.
  if (recon_geometry_map.size() == 1) {
    fused_points3d = recon_geometry_map.begin()->second->points3d;
    return true;
  }

  auto target_corners = sensor_manager_->GetTargetCorners();

  if (!target_corners) {
    spdlog::error(
        "Target corners not available for reconstruction points fusion.");
    return false;
  }

  // Gather all 3D observations in all reconstructions for each landmark ID.
  std::map<int, std::vector<Eigen::Vector3d>> landmark_observations;
  for (const auto& [label, recon_geometry] : recon_geometry_map) {
    for (const auto& [landmark_id, point3d] : recon_geometry->points3d) {
      landmark_observations[landmark_id].push_back(point3d);
    }
  }

  // Fuse observations for each landmark ID.
  for (const auto& [landmark_id, observations] : landmark_observations) {
    if (observations.empty()) continue;

    // Fuse observations by averaging.
    Eigen::Vector3d fused_point = Eigen::Vector3d::Zero();
    for (const auto& obs : observations) {
      fused_point += obs;
    }
    fused_point /= observations.size();

    fused_points3d[landmark_id] = fused_point;

    // Check consistency
    // Get target index for the current landmark ID.
    int target_idx = target_corners->GetTargetIdx(landmark_id);

    if (target_idx == -1) {
      spdlog::error(
          "Landmark ID {} does not belong to any known target during fusion.",
          landmark_id);
      return false;
    }

    const double kDistThreshold =
        sensor_manager_->GetAllTargetConfigs().at(target_idx).fiducial_size /
        5.0;  // 20% of fiducial size
    double dist_sum = 0.0;
    for (const auto& obs : observations) {
      double dist = (obs - fused_point).norm();
      dist_sum += dist;
    }

    double dist = dist_sum / observations.size();
    if (dist > kDistThreshold) {
      spdlog::warn(
          "Landmark {} of target {} has inconsistent observations during "
          "fusion (dist = {:.3f} > {:.3f}).",
          landmark_id, target_idx, dist, kDistThreshold);
    }
  }

  return true;
}

bool SfmCalibrator::ComputeRelativeTargetPoses(
    const std::map<int, Eigen::Vector3d>& fused_points3d,
    std::map<int, Eigen::Matrix4d>& T_T0_Ti_map) {
  const auto& target_configs = sensor_manager_->GetAllTargetConfigs();
  if (target_configs.empty()) {
    spdlog::error(
        "No target configurations found for relative pose computation.");
    return false;
  }

  if (target_configs.size() == 1) {
    // Only one target, set identity pose.
    T_T0_Ti_map[target_configs.begin()->first] = Eigen::Matrix4d::Identity();
    return true;
  }

  // ===========================================================================

  // Step 1: Prepare the various data structures for pose computation.
  std::vector<int> target_indices;
  for (const auto& [target_idx, config] : target_configs) {
    target_indices.push_back(target_idx);
  }

  std::map<int, Eigen::Matrix4d> T_W_Ti_map;
  const auto& target_corners = sensor_manager_->GetTargetCorners();
  if (!target_corners) {
    spdlog::error(
        "Target corners not available for relative pose computation.");
    return false;
  }

  // ===========================================================================

  // Step 2: Compute absolute poses for each target in reconstruction world
  // frame using Umeyama alignment.
  for (size_t t = 0; t < target_indices.size(); ++t) {
    // Step 2.1: Gather corresponding points between fused reconstruction and
    // ideal target corners.
    int target_idx = target_indices[t];
    const auto& range = target_corners->target_id_ranges.at(target_idx);
    int start_id = range.first;
    int end_id = range.second;

    std::vector<Eigen::Vector3d> fused_points;  // Fused reconstructed points.
    std::vector<Eigen::Vector3d> ideal_points;  // Ideal target points.

    for (int id = start_id; id <= end_id; ++id) {
      // Check if this landmark exists in both fused points and ideal target
      // corners.
      auto fused_it = fused_points3d.find(id);
      if (fused_it == fused_points3d.end()) continue;

      if (!target_corners->Contains(id)) {
        spdlog::error(
            "Landmark ID {} not found in target corners in relative pose "
            "computation.",
            id);
        return false;
      }

      const auto& corner = target_corners->At(id);
      ideal_points.push_back(corner.position_local);
      fused_points.push_back(fused_it->second);
    }

    const size_t kMinPointsForUmeyama = 6;
    if (fused_points.size() < kMinPointsForUmeyama) {
      spdlog::error(
          "Insufficient points ({}) for target {} pose computation. Need at "
          "least {}.",
          fused_points.size(), target_idx, kMinPointsForUmeyama);
      return false;
    }

    // Step 2.2: Compute Umeyama alignment between ideal target points and
    // fused reconstructed points.
    // Convert to Eigen matrices for Umeyama alignment.
    Eigen::MatrixXd fused_mat(3, fused_points.size());
    Eigen::MatrixXd ideal_mat(3, ideal_points.size());
    for (size_t i = 0; i < fused_points.size(); ++i) {
      fused_mat.col(i) = fused_points[i];
      ideal_mat.col(i) = ideal_points[i];
    }

    // Compute Umeyama alignment. (Without scaling since we already scaled the
    // reconstruction)
    // The resulting matrix transforms ideal target points (in local frame Ti)
    // to fused reconstructed points (in reconstruction world frame W).
    Eigen::Matrix4d T_W_Ti = Eigen::umeyama(ideal_mat, fused_mat, false);

    // Step 2.3: Validate the transformation by computing reprojection error.
    double total_error = 0.0;
    for (size_t i = 0; i < ideal_points.size(); ++i) {
      Eigen::Vector4d ideal_point;
      ideal_point << ideal_points[i], 1.0;
      Eigen::Vector3d transformed = (T_W_Ti * ideal_point).head<3>();
      total_error += (transformed - fused_points[i]).norm();
    }
    double avg_error = total_error / ideal_points.size();

    // Get fiducial size for this target.
    double fiducial_size = target_configs.at(target_idx).fiducial_size;
    const double kMaxAvgError = fiducial_size / 10.0;  // 10% of fiducial size.

    if (avg_error > kMaxAvgError) {
      spdlog::warn(
          "Target {}: Average alignment error ({:.4f}m) exceeds threshold "
          "({:.4f}m). Pose may be inaccurate.",
          target_idx, avg_error, kMaxAvgError);
    }

    T_W_Ti_map[target_idx] = T_W_Ti;
  }

  // ===========================================================================

  // Step 3: Compute relative poses between targets based on absolute poses.
  // Compute relative poses: T_T0_Ti = T_W_T0^{-1} * T_W_Ti
  int base_target_idx = target_indices[0];
  if (T_W_Ti_map.find(base_target_idx) == T_W_Ti_map.end()) {
    spdlog::error("Base target {} pose not computed.", base_target_idx);
    return false;
  }

  Eigen::Matrix4d T_W_T0_inv = T_W_Ti_map[base_target_idx].inverse();
  for (const auto& target_idx : target_indices) {
    if (T_W_Ti_map.find(target_idx) == T_W_Ti_map.end()) {
      spdlog::error("Target {} pose not computed.", target_idx);
      return false;
    }
    if (target_idx == base_target_idx) continue;
    T_T0_Ti_map[target_idx] = T_W_T0_inv * T_W_Ti_map[target_idx];
  }

  // ===========================================================================

  return true;
}

void SfmCalibrator::PrintCalibrationResults() {
  spdlog::info("Base camera: {}, world target: {}",
               calib_parameters_->base_camera_label,
               calib_parameters_->world_target_index);
  // Print camera calibration results.
  for (const auto& [label, _] : sensor_manager_->GetAllCamConfigs()) {
    spdlog::info(" - Results for camera: {}", label);

    Eigen::Vector3d trans_Cb_Ci = calib_parameters_->trans_Cb_Ci.at(label);
    Eigen::Quaterniond rot_Cb_Ci =
        calib_parameters_->rot_Cb_Ci.at(label).unit_quaternion();
    CamIntrinsicBase::Ptr cam_intrinsic =
        calib_parameters_->cam_intrinsics.at(label);
    spdlog::info("Translation Ci in Cb: [{:.6f}, {:.6f}, {:.6f}]",
                 trans_Cb_Ci.x(), trans_Cb_Ci.y(), trans_Cb_Ci.z());
    spdlog::info(
        "Rotation from Ci to Cb (quat_xyzw): [{:.6f}, {:.6f}, {:.6f}, "
        "{:.6f}]",
        rot_Cb_Ci.x(), rot_Cb_Ci.y(), rot_Cb_Ci.z(), rot_Cb_Ci.w());
    spdlog::info("Intrinsic parameters: {}",
                 cam_intrinsic->IntrinsicsToString());
  }

  // Print target relative poses.
  for (const auto& [idx, _] : sensor_manager_->GetAllTargetConfigs()) {
    spdlog::info(" - Results for target: {}", idx);

    Eigen::Vector3d trans_W_Ti = calib_parameters_->trans_W_Ti.at(idx);
    Eigen::Quaterniond rot_W_Ti =
        calib_parameters_->rot_W_Ti.at(idx).unit_quaternion();
    spdlog::info("Translation Ti in W: [{:.6f}, {:.6f}, {:.6f}]",
                 trans_W_Ti.x(), trans_W_Ti.y(), trans_W_Ti.z());
    spdlog::info(
        "Rotation from Ti to W (quat_xyzw): [{:.6f}, {:.6f}, {:.6f}, {:.6f}]",
        rot_W_Ti.x(), rot_W_Ti.y(), rot_W_Ti.z(), rot_W_Ti.w());
  }
}

}  // namespace xr_ucalib
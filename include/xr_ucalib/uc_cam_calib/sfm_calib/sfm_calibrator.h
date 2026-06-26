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

// clang-format off
#include <memory>
#include <map>
#include <string>

#include <colmap/scene/database.h>
#include <colmap/scene/reconstruction.h>

#include "xr_ucalib/uc_common/calib_parameter/calib_parameters.h"
#include "xr_ucalib/uc_common/config/system_config.h"
#include "xr_ucalib/uc_common/sensor_data/sensor_manager.h"
#include "xr_ucalib/uc_cam_calib/sfm_calib/reconstruction_geometry.h"
// clang-format on

namespace xr_ucalib {

/**
 * @brief COLMAP-based multi-camera calibrator used primarily for system
 * initialization.
 *
 * Workflow:
 * 1. Incremental Sfm: Perform structure form motion (SFM) for each camera
 * using target corners as independent landmarks to initialize  cam intrinsics.
 * 2. Map Fusion & Scaling: Merge individual maps and recovers physical scale
 * using target geometry to initialize extrinsic parameters between cameras (if
 * multiple cameras).
 * 3. Target Alignment: Initialize extrinsics between multiple calibration
 * targets based on spatial priors (if multiple targets).
 *
 * Result: Provides reliable initial values for camera intrinsics, extrinsics,
 * and multi-target transformations.
 */
class SfmCalibrator {
 public:
  using Ptr = std::shared_ptr<SfmCalibrator>;

  // Factory method to create a shared pointer instance.
  static Ptr Create(const SystemConfig::Ptr& system_config,
                    const SensorManager::Ptr& sensor_manager,
                    const CalibParameters::Ptr& calib_parameters) {
    return Ptr(
        new SfmCalibrator(system_config, sensor_manager, calib_parameters));
  }

  // Run the SFM calibration process for all cameras, initializing camera
  // intrinsics, camera extrinsics, and target transformations.
  bool RunCalibration();

 private:
  SfmCalibrator(const SystemConfig::Ptr& system_config,
                const SensorManager::Ptr& sensor_manager,
                const CalibParameters::Ptr& calib_parameters)
      : system_config_(system_config),
        sensor_manager_(sensor_manager),
        calib_parameters_(calib_parameters) {}

  // Type aliases for COLMAP.
  // COLMAP reconstruction pointer.
  using ReconstructionPtr = std::shared_ptr<colmap::Reconstruction>;
  // Map from COLMAP image IDs to our CamFrame pointers.
  using ImageMap = std::map<colmap::image_t, CamFrame::Ptr>;
  // Map from our camera label to COLMAP reconstruction pointer.
  using ReconMap = std::map<std::string, ReconstructionPtr>;
  // Map from our camera label to ReconstructionGeometry pointer.
  using ReconGeometryMap = std::map<std::string, ReconstructionGeometry::Ptr>;

  /**
   * @brief Construct COLMAP database for a specific camera sequence.
   *
   * For each camera frame, we first add the camera to the database, then add
   * images and keypoints. Finally, we match keypoints between image pairs based
   * on landmark IDs and write the matches to the database.
   *
   * @param[in] label Unique label of the camera.
   * @param[in] cam_seq Camera sequence containing frames and keypoints.
   * @param[in] db_path Path to the constructed COLMAP database.
   * @param[out] img_map Map from engaged COLMAP image IDs to camera frames.
   * @return true If construction is successful, false otherwise.
   */
  bool ConstructDatabase(const std::string& label,
                         const CamSequence::Ptr& cam_seq,
                         const std::string& db_path, ImageMap& img_map);

  /**
   * @brief Run COLMAP incremental mapper based on the provided database
   * to perform SFM for a camera.
   *
   * @param[in] db_path Path to the COLMAP database.
   * @param[in] img_map Map from engaged COLMAP image IDs to camera frames.
   * @param[in] work_dir Working directory for COLMAP mapper outputs.
   * @param[out] reconstruction Output COLMAP reconstruction pointer to store
   * results.
   * @return true If mapping is successful, false otherwise.
   */
  bool RunMapper(const std::string& db_path, const ImageMap& img_map,
                 const std::string& work_dir,
                 ReconstructionPtr& reconstruction);

  /**
   * @brief Initialize COLMAP camera model from given camera intrinsic
   * parameters.
   *s
   * @param[in] cam_intrinsic Camera intrinsic parameters.
   * @param[in] colmap_cam Output COLMAP camera model to be initialized.
   * @return[out] true If initialization is successful, false otherwise.
   */
  bool InitialColmapCamera(const CamIntrinsicBase::Ptr& cam_intrinsic,
                           colmap::Camera& colmap_cam);

  /**
   * @brief Match keypoints between two camera frames based on landmark IDs to
   * get COLMAP feature matches.
   *
   * Each match stores the zero-based indices of the keypoints as they appear
   * in the CamFrame's keypoint container. These indices must correspond to the
   * order in which keypoints are exported to the COLMAP database.
   *
   * @param[in] cam_frame1 Camera frame 1.
   * @param[in] cam_frame2 Camera frame 2.
   * @return * colmap::FeatureMatches Feature matches between the two frames.
   */
  colmap::FeatureMatches MatchTwoFrames(const CamFrame::Ptr& cam_frame1,
                                        const CamFrame::Ptr& cam_frame2);

  /**
   * @brief Convert COLMAP reconstruction to our ReconstructionGeometry format
   * and scale it to real-world units based on the target size.
   *
   * @param reconstruction COLMAP reconstruction result.
   * @param label Camera label.
   * @param img_map Map from COLMAP image IDs to camera frames.
   * @param recon_geometry Output scaled reconstruction geometry.
   * @return true If conversion and scaling is successful, false otherwise.
   */
  bool ConvertAndScaleReconstruction(
      const ReconstructionPtr& reconstruction, const std::string& label,
      const ImageMap& img_map, ReconstructionGeometry::Ptr& recon_geometry);

  /**
   * @brief Fuse 3D points from multiple reconstruction geometries into a single
   * set.
   *
   * @param recon_geometry_map Reconstruction geometries from multiple cameras.
   * @param fused_points3d Fused 3D points output map: Key - our unified
   * landmark ID, Value - 3D point.
   * @return true If fusion is successful, false otherwise.
   */
  bool FuseReconstructionPoints(const ReconGeometryMap& recon_geometry_map,
                                std::map<int, Eigen::Vector3d>& fused_points3d);

  /**
   * @brief Compute relative target poses based on fused 3D points and known
   * target information.
   *
   * This is mainly used to initialize the transformations between multiple
   * targets. We compute the transformation from target 0 to target i for all.
   *
   * @param fused_points3d Fused 3D points map: Key - our unified landmark ID,
   * Value - 3D point.
   * @param T_T0_Ti_map Output map of relative target poses: Key - target
   * index, Value - transformation from target 0 to target i.
   * @return true If computation is successful, false otherwise.
   */
  bool ComputeRelativeTargetPoses(
      const std::map<int, Eigen::Vector3d>& fused_points3d,
      std::map<int, Eigen::Matrix4d>& T_T0_Ti_map);

  /// @brief Print the calibration results after SFM calibration.
  void PrintCalibrationResults();

  // Pointers to system configuration, sensor manager, and calibration
  // parameters.
  SystemConfig::Ptr system_config_;
  SensorManager::Ptr sensor_manager_;
  CalibParameters::Ptr calib_parameters_;
};

}  // namespace xr_ucalib
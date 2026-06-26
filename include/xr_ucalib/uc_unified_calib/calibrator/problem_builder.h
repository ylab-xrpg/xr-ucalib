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

#include <ceres/ceres.h>

#include <memory>
#include <string>

#include "xr_ucalib/uc_common/calib_parameter/calib_parameters.h"
#include "xr_ucalib/uc_common/config/system_config.h"
#include "xr_ucalib/uc_common/sensor_data/sensor_manager.h"
#include "xr_ucalib/uc_unified_calib/spline/spline_bundle.hpp"

namespace xr_ucalib {

/// @brief Builder class to construct Ceres problem for unified calibration.
class ProblemBuilder {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<ProblemBuilder>;

  // Spline configuration.
  static int constexpr kSplineOrder = UnifiedCalibConfig::kSplineOrder;
  using SplineBundleType = SplineBundle<kSplineOrder>;
  using SplineMetaType = SplineMeta<kSplineOrder>;

  // Context to hold all shared data.
  struct Context {
    // System configuration.
    SystemConfig::Ptr system_config;

    // Calibration parameters that we want to optimize.
    CalibParameters::Ptr calib_parameters;

    // Spline information and spline bundle for the system continuous-time
    // trajectory.
    SplineBundleType::Ptr spline_bundle;
    SplineInfo trans_spline_info;
    SplineInfo rot_spline_info;
  };

  // Factory method to create a shared pointer instance.
  static Ptr Create(const Context& context) {
    return Ptr(new ProblemBuilder(context));
  }

  /**
   * @brief Add camera reprojection residuals to the Ceres problem.
   *
   * @param[out] problem Ceres problem to which residuals will be added.
   * @param[in] label Sensor label of the camera.
   * @param[in] cam_frame Camera frame containing observations.
   * @param[in] target_corners 3D target corners.
   * @param[in] cam_config Camera configuration.
   * @param[in] target_configs Map of target configurations.
   * @return true If residuals are successfully added.
   */
  bool AddCamReprojResiduals(ceres::Problem& problem, const std::string& label,
                             const CamFrame::Ptr& cam_frame,
                             const TargetCorner3D::Ptr& target_corners,
                             const CamConfig& cam_config,
                             const std::map<int, TargetConfig>& target_configs);

  /**
   * @brief Add IMU accelerometer residuals to the Ceres problem.
   *
   * @param[out] problem Ceres problem to which residuals will be added.
   * @param[in] label Sensor label of the IMU.
   * @param[in] imu_frame IMU frame containing measurements.
   * @param[in] imu_config IMU configuration.
   * @return true If residuals are successfully added.
   */
  bool AddImuAccResiduals(ceres::Problem& problem, const std::string& label,
                          const ImuFrame::Ptr& imu_frame,
                          const ImuConfig& imu_config);

  /**
   * @brief Add IMU gyroscope residuals to the Ceres problem.
   *
   * @param[out] problem Ceres problem to which residuals will be added.
   * @param[in] label Sensor label of the IMU.
   * @param[in] imu_frame IMU frame containing measurements.
   * @param[in] imu_config IMU configuration.
   * @return true If residuals are successfully added.
   */
  bool AddImuGyrResiduals(ceres::Problem& problem, const std::string& label,
                          const ImuFrame::Ptr& imu_frame,
                          const ImuConfig& imu_config);

  /**
   * @brief Add magnetometer residuals to the Ceres problem.
   *
   * @param[out] problem Ceres problem to which residuals will be added.
   * @param[in] label Sensor label of the magnetometer.
   * @param[in] mag_frame Magnetometer frame containing measurements.
   * @param[in] mag_config Magnetometer configuration.
   * @return true If residuals are successfully added.
   */
  bool AddMagResiduals(ceres::Problem& problem, const std::string& label,
                       const MagFrame::Ptr& mag_frame,
                       const MagConfig& mag_config);

 private:
  explicit ProblemBuilder(const Context& context) : context_(context) {}

  /**
   * @brief Given a timestamp of sensor measurement (in the body frame's clock),
   * return the time range of B-spline meta data. Ensure B-spline data
   * corresponding to sensor measurements can be obtained when the time offset
   * changes during the optimization.
   *
   * @param[in] meas_time
   * @param[out] min_time
   * @param[out] max_time
   * @return true If the time range is successfully calculated.
   */
  bool CalMetaMinMaxTime(const double& meas_time, double& min_time,
                         double& max_time);

  /**
   * @brief Add R(3) spline knot data to the Ceres problem and parameter block
   * vector.
   *
   * @param[in] spline The R(3) spline containing knot data.
   * @param[in] spline_meta The spline meta data.
   * @param[out] problem Ceres problem to which parameter blocks will be added.
   * @param[out] param_block_vector Vector to store pointers to parameter
   * blocks.
   */
  void AddR3dKnotData(const SplineBundleType::R3dSplineType& spline,
                      const SplineMetaType& spline_meta,
                      ceres::Problem& problem,
                      std::vector<double*>& param_block_vector);

  /**
   * @brief Add SO(3) spline knot data to the Ceres problem and parameter block
   * vector.
   *
   * @param[in] spline The SO(3) spline containing knot data.
   * @param[in] spline_meta The spline meta data.
   * @param[out] problem Ceres problem to which parameter blocks will be added.
   * @param[out] param_block_vector Vector to store pointers to parameter
   * blocks.
   */
  void AddSo3dKnotData(const SplineBundleType::So3dSplineType& spline,
                       const SplineMetaType& spline_meta,
                       ceres::Problem& problem,
                       std::vector<double*>& param_block_vector);

  // Ceres manifolds and loss functions.
  // Note: Memory is managed by unique_ptr. Use ceres::DO_NOT_TAKE_OWNERSHIP
  // when constructing ceres::Problem to avoid double deletion.
  std::unique_ptr<ceres::Manifold> quat_manifold_ =
      std::make_unique<ceres::QuaternionManifold>();
  std::unique_ptr<ceres::Manifold> sphere_manifold_ =
      std::make_unique<ceres::SphereManifold<3>>();
  std::unique_ptr<ceres::LossFunction> huber_loss_function_ =
      std::make_unique<ceres::HuberLoss>(1.0);

  // Context holding all configuration and state resources
  Context context_;
};

}  // namespace xr_ucalib

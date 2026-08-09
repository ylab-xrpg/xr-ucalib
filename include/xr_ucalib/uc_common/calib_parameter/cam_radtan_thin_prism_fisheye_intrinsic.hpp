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

#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sophus/se3.hpp>

#include "xr_ucalib/uc_common/calib_parameter/cam_intrinsic_base.h"

namespace xr_ucalib {

/**
 * @brief Camera intrinsics for the FisheyeRadTanThinPrism model.
 *
 * The model is commonly called Fisheye624 because it combines six radial,
 * two tangential, and four thin-prism distortion coefficients. XR-UCalib uses
 * the serialized parameter order
 * [fx, fy, cx, cy, k0-k5, p0, p1, s0-s3].
 */
class CamRadTanThinPrismFisheyeIntrinsic : public CamIntrinsicBase {
 public:
  using Ptr = std::shared_ptr<CamRadTanThinPrismFisheyeIntrinsic>;

  /// @brief Create a full Fisheye624 or restricted Fisheye620 model.
  static Ptr Create(
      CamModelType type = CamModelType::RAD_TAN_THIN_PRISM_FISHEYE) {
    return Ptr(new CamRadTanThinPrismFisheyeIntrinsic(type));
  }

  // Accessors follow the serialized parameter order documented above.
  double& fx() { return parameters[0]; }
  double& fy() { return parameters[1]; }
  double& cx() { return parameters[2]; }
  double& cy() { return parameters[3]; }
  double& k0() { return parameters[4]; }
  double& k1() { return parameters[5]; }
  double& k2() { return parameters[6]; }
  double& k3() { return parameters[7]; }
  double& k4() { return parameters[8]; }
  double& k5() { return parameters[9]; }
  double& p0() { return parameters[10]; }
  double& p1() { return parameters[11]; }
  double& s0() { return parameters[12]; }
  double& s1() { return parameters[13]; }
  double& s2() { return parameters[14]; }
  double& s3() { return parameters[15]; }

  /** @brief Project a world point into the image. */
  template <typename T>
  static void Space2Image(const Eigen::Matrix<T, 3, 1>& trans_W_C,
                          const Eigen::Quaternion<T>& rot_W_C,
                          const Eigen::Matrix<T, 3, 1>& point_in_W,
                          const T* const params,
                          Eigen::Matrix<T, 2, 1>& point_2d) {
    const Eigen::Matrix<T, 3, 1> point_in_C =
        rot_W_C.inverse() * (point_in_W - trans_W_C);
    Cam2Image(point_in_C, params, point_2d);
  }

  /** @brief Project a camera-frame point into the image. */
  template <typename T>
  static void Cam2Image(const Eigen::Matrix<T, 3, 1>& point_in_C,
                        const T* const params,
                        Eigen::Matrix<T, 2, 1>& point_2d) {
    T u = point_in_C.x() / point_in_C.z();
    T v = point_in_C.y() / point_in_C.z();

    // Convert normalized pinhole coordinates to polar-angle coordinates.
    const T radius = ceres::sqrt(u * u + v * v);
    if (radius > T(std::numeric_limits<double>::epsilon())) {
      const T theta_div_radius = ceres::atan(radius) / radius;
      u *= theta_div_radius;
      v *= theta_div_radius;
    }

    T du;
    T dv;
    Distortion(&params[4], u, v, &du, &dv);
    point_2d.x() = params[0] * (u + du) + params[2];
    point_2d.y() = params[1] * (v + dv) + params[3];
  }

  /**
   * @brief Invert an image point to normalized pinhole coordinates.
   *
   * @return true if Newton iteration converges to a finite forward-facing ray.
   */
  static bool Image2Cam(const Eigen::Vector2d& point_2d,
                        const double* const params,
                        Eigen::Vector2d& point_in_cam) {
    if (params == nullptr || !point_2d.allFinite() ||
        !std::isfinite(params[0]) || !std::isfinite(params[1]) ||
        params[0] == 0.0 || params[1] == 0.0) {
      return false;
    }

    const Eigen::Vector2d distorted((point_2d.x() - params[2]) / params[0],
                                    (point_2d.y() - params[3]) / params[1]);
    if (!distorted.allFinite()) {
      return false;
    }

    constexpr size_t kMaxNumIterations = 100;
    constexpr double kMaxStepNorm = 1e-10;
    constexpr double kRelativeStepSize = 1e-6;
    Eigen::Vector2d solution = distorted;
    bool converged = false;

    for (size_t i = 0; i < kMaxNumIterations; ++i) {
      const double step_x =
          std::max(std::numeric_limits<double>::epsilon(),
                   std::abs(kRelativeStepSize * solution.x()));
      const double step_y =
          std::max(std::numeric_limits<double>::epsilon(),
                   std::abs(kRelativeStepSize * solution.y()));

      Eigen::Vector2d delta;
      Eigen::Vector2d delta_x_backward;
      Eigen::Vector2d delta_x_forward;
      Eigen::Vector2d delta_y_backward;
      Eigen::Vector2d delta_y_forward;
      Distortion(&params[4], solution.x(), solution.y(), &delta.x(),
                 &delta.y());
      Distortion(&params[4], solution.x() - step_x, solution.y(),
                 &delta_x_backward.x(), &delta_x_backward.y());
      Distortion(&params[4], solution.x() + step_x, solution.y(),
                 &delta_x_forward.x(), &delta_x_forward.y());
      Distortion(&params[4], solution.x(), solution.y() - step_y,
                 &delta_y_backward.x(), &delta_y_backward.y());
      Distortion(&params[4], solution.x(), solution.y() + step_y,
                 &delta_y_forward.x(), &delta_y_forward.y());

      Eigen::Matrix2d jacobian;
      jacobian(0, 0) =
          1.0 + (delta_x_forward.x() - delta_x_backward.x()) / (2.0 * step_x);
      jacobian(0, 1) =
          (delta_y_forward.x() - delta_y_backward.x()) / (2.0 * step_y);
      jacobian(1, 0) =
          (delta_x_forward.y() - delta_x_backward.y()) / (2.0 * step_x);
      jacobian(1, 1) =
          1.0 + (delta_y_forward.y() - delta_y_backward.y()) / (2.0 * step_y);
      if (!jacobian.allFinite() || !delta.allFinite()) {
        return false;
      }

      const Eigen::FullPivLU<Eigen::Matrix2d> decomposition(jacobian);
      if (!decomposition.isInvertible()) {
        return false;
      }
      const Eigen::Vector2d step =
          decomposition.solve(solution + delta - distorted);
      if (!step.allFinite()) {
        return false;
      }
      solution -= step;
      if (!solution.allFinite()) {
        return false;
      }
      if (step.norm() < kMaxStepNorm) {
        converged = true;
        break;
      }
    }

    if (!converged) {
      return false;
    }

    // Convert angular coordinates back to normalized pinhole coordinates.
    const double theta = solution.norm();
    if (theta > std::numeric_limits<double>::epsilon()) {
      const double theta_cos_theta = theta * std::cos(theta);
      if (!std::isfinite(theta_cos_theta) ||
          theta_cos_theta <= std::numeric_limits<double>::epsilon()) {
        return false;
      }
      solution *= std::sin(theta) / theta_cos_theta;
    }
    if (!solution.allFinite()) {
      return false;
    }

    point_in_cam = solution;
    return true;
  }

 private:
  explicit CamRadTanThinPrismFisheyeIntrinsic(CamModelType type) {
    cam_model_type = type;
    parameter_size = 16;
    parameters.resize(parameter_size, 0.0);
  }

  /** @brief Evaluate radial, tangential, and thin-prism displacement. */
  template <typename T>
  static void Distortion(const T* const extra_params, const T u, const T v,
                         T* const du, T* const dv) {
    const T theta2 = u * u + v * v;
    T radial = T(1);
    T theta_power = T(1);
    for (int i = 0; i < 6; ++i) {
      theta_power *= theta2;
      radial += extra_params[i] * theta_power;
    }

    const T x = radial * u;
    const T y = radial * v;
    const T x2 = x * x;
    const T y2 = y * y;
    const T xy = x * y;
    const T radius2 = x2 + y2;
    const T radius4 = radius2 * radius2;
    const T dx_tang =
        T(2) * extra_params[7] * xy + extra_params[6] * (radius2 + T(2) * x2);
    const T dy_tang =
        T(2) * extra_params[6] * xy + extra_params[7] * (radius2 + T(2) * y2);
    const T dx_prism = extra_params[8] * radius2 + extra_params[9] * radius4;
    const T dy_prism = extra_params[10] * radius2 + extra_params[11] * radius4;

    *du = x + dx_tang + dx_prism - u;
    *dv = y + dy_tang + dy_prism - v;
  }
};

}  // namespace xr_ucalib

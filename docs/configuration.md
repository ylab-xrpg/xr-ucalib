# XR-UCalib Configuration Reference

XR-UCalib reads calibration settings from `input_config.json`. Start from the
[example configuration](../data/test_data_handheld/input_config.json) and refer
to this page for field defaults, units, and constraints. The main build and run
instructions remain in the [project README](../README.md).

Fields omitted from the JSON use the defaults listed below. Every system must
contain at least one camera and one target, with exactly one camera marked as
the base camera. Unified calibration additionally requires at least one IMU and
exactly one IMU marked as the body frame.

## Calibration Stages

`cam_calib_config` controls the camera-only initialization and refinement:

| Field | Default | Description |
| --- | ---: | --- |
| `enable_ba_refine` | `true` | Run camera-rig bundle adjustment after per-camera SfM. |
| `cam_down_sample_rate` | `1` | Keep one synchronized camera frame out of every N frames; must be positive. |
| `multi_thread_num` | `8` | Worker threads used by camera calibration; must be positive. |
| `ceres_max_iterations` | `20` | Maximum Ceres iterations for camera-rig refinement; must be positive. |

`unified_calib_config` controls continuous-time camera/IMU/magnetometer
calibration:

| Field | Default | Description |
| --- | ---: | --- |
| `enable_unified_calib` | `false` | Run unified calibration after camera calibration. When disabled, IMU and magnetometer configurations are ignored. |
| `spline_knot_interval` | `0.01` | Continuous-time trajectory knot interval in seconds; must be positive. |
| `max_toff_change` | `0.1` | Maximum permitted change of an optimized time offset in seconds; must be non-negative. |
| `gravity_magnitude` | `9.8` | Gravity magnitude in m/s²; must be positive. |
| `fix_camera_intrinsics` | `true` | Keep camera intrinsics fixed during unified calibration. Camera-only calibration can still estimate them first. |
| `fix_camera_extrinsics` | `true` | Keep non-base camera extrinsics fixed during unified calibration. |
| `multi_thread_num` | `8` | Worker threads used by unified calibration; must be positive. |
| `ceres_max_iterations` | `20` | Maximum Ceres iterations for unified calibration; must be positive. |

## Camera Configuration

Each entry in `cam_configs` describes one camera directory under `sensor_data`:

| Field | Default | Description |
| --- | ---: | --- |
| `file_name` | empty | Camera directory name and unique label, for example `cam0`. |
| `cam_model_type` | `RADTAN` | Projection model; supported values are listed below. |
| `detection_display_mode` | `NONE` | AprilTag visualization mode: `NONE`, `STEP`, or `CONTINUOUS`. |
| `save_all_reproj_image` | `false` | Save every per-frame reprojection image instead of a uniform sample. |
| `reproj_threshold` | `-1` | Camera-rig outlier threshold in pixels. A negative value disables reprojection outlier rejection. |
| `down_sample_rate_ucalib` | `1` | Keep one frame out of every N frames during unified calibration; must be positive. |
| `initial_focal_length` | `-1` | Initial focal-length estimate in pixels. A finite positive value is required. |
| `noise` | `0.5` | Camera measurement standard deviation in pixels; must be positive. |
| `base_camera_flag` | `false` | Select the base camera frame `Cb`; exactly one camera must enable this. |
| `fix_temporal_extrinsic` | `false` | Fix this camera's time offset to `toff_Cb_Ci_prior`. |
| `fix_spatial_extrinsic` | `false` | Fix this camera's translation and rotation to their priors. |
| `fix_intrinsic` | `false` | Fix all intrinsic parameters to `intrinsic_prior`. |
| `toff_Cb_Ci_prior` | `0` | Camera time-offset prior in seconds. For the base camera this represents `B <- Cb`; otherwise `Cb <- Ci`. |
| `trans_Cb_Ci_prior` | zero | Translation prior as an `{x, y, z}` object, in meters. |
| `rot_q_Cb_Ci_prior` | identity | Rotation prior as an `{x, y, z, w}` quaternion object. |
| `intrinsic_prior` | model-dependent | Full intrinsic vector required when `fix_intrinsic` is `true`. |

Supported camera models and intrinsic vector order:

| `cam_model_type` | Parameter order |
| --- | --- |
| `RADTAN` | `[fx, fy, cx, cy, k1, k2, p1, p2]` |
| `EQUIDISTANT` | `[fx, fy, cx, cy, k1, k2, k3, k4]` |
| `RAD_TAN_THIN_PRISM_FISHEYE` | `[fx, fy, cx, cy, k0, k1, k2, k3, k4, k5, p0, p1, s0, s1, s2, s3]` |
| `RAD_TAN_THIN_PRISM_FISHEYE_620` | Same 16-value storage order as above; `s0`–`s3` are forced to zero and held constant. |

The last two models are referred to as Fisheye624 and Fisheye620. Both use a
16-value serialized parameter block so that JSON output and Ceres parameter
storage remain consistent. If a fixed Fisheye620 prior is supplied, its final
four values must therefore be zero.

## IMU Configuration

Each entry in `imu_configs` describes one CSV file under `sensor_data`:

| Field | Default | Description |
| --- | ---: | --- |
| `file_name` | empty | IMU CSV filename and unique label. |
| `imu_model_type` | `CALIBRATED` | One of `CALIBRATED`, `SCALE`, `MISALIGN`, or `SCALE_MISALIGN`. |
| `down_sample_rate_ucalib` | `1` | Keep one measurement out of every N samples; must be positive. |
| `frequency_hz` | `200` | Nominal sampling frequency in Hz; must be positive. |
| `noise` | implementation default | Continuous-time standard deviations `{acc_n, acc_b, gyr_n, gyr_b}`; all values must be positive. |
| `body_frame_flag` | `false` | Select the system body frame `B`; exactly one IMU must enable this in unified mode. |
| `fix_temporal_extrinsic` | `false` | Fix the IMU time offset to `toff_B_Ii_prior`. |
| `fix_spatial_extrinsic` | `false` | Fix the IMU translation and rotation to their priors. |
| `toff_B_Ii_prior` | `0` | Time offset from IMU clock `Ii` to body clock `B`, in seconds. |
| `trans_B_Ii_prior` | zero | IMU translation prior as an `{x, y, z}` object, in meters. |
| `rot_q_B_Ii_prior` | identity | Rotation from `Ii` to `B` as an `{x, y, z, w}` quaternion. |
| `acc_bias_prior` | zero | Accelerometer bias prior as an `{x, y, z}` object. |
| `gyr_bias_prior` | zero | Gyroscope bias prior as an `{x, y, z}` object. |

The selected body IMU defines `B`, so its spatial and temporal extrinsics are
fixed to identity and zero. `CALIBRATED` assumes scale and axis alignment are
known; the other model choices enable the corresponding intrinsic terms.

## Magnetometer Configuration

Each entry in `mag_configs` describes one CSV file under `sensor_data`:

| Field | Default | Description |
| --- | ---: | --- |
| `file_name` | empty | Magnetometer CSV filename and unique label. |
| `down_sample_rate_ucalib` | `1` | Keep one measurement out of every N samples; must be positive. |
| `noise` | `0.05` | Standard deviation of the normalized measurement; must be positive. |
| `fix_temporal_extrinsic` | `false` | Fix the time offset to `toff_B_Mi_prior`. |
| `fix_spatial_extrinsic` | `false` | Fix the rotation to `rot_q_B_Mi_prior`. |
| `toff_B_Mi_prior` | `0` | Time offset from magnetometer clock `Mi` to body clock `B`, in seconds. |
| `rot_q_B_Mi_prior` | identity | Rotation from `Mi` to `B` as an `{x, y, z, w}` quaternion. |

Magnetometer translation is not estimated. Measurements must be intrinsically
calibrated before use and should be close to unit length; the loader normalizes
accepted measurements before calibration.

## Target Configuration

Each entry in `target_configs` describes one AprilTag grid:

| Field | Default | Description |
| --- | ---: | --- |
| `fiducial_type` | `APRILTAG` | Fiducial detector type; currently only `APRILTAG` is supported. |
| `target_idx` | `-1` | Unique non-negative target index. The target with the smallest index defines the world frame `W`. |
| `fiducial_size` | `0.088` | Tag edge length in meters; must be positive. |
| `fiducial_spacing` | `0.02` | Ratio of the gap between neighboring tags to `fiducial_size`; must be non-negative. |
| `fiducial_rows` | `6` | Number of tag rows; must be positive. |
| `fiducial_cols` | `6` | Number of tag columns; must be positive. |
| `start_id` | `0` | First corner ID assigned to the grid. Each tag consumes four consecutive corner IDs. ID ranges of different targets must not overlap. |
| `fix_spatial_extrinsic` | `false` | Fix this target pose to `trans_W_T_prior` and `rot_q_W_T_prior`. |
| `trans_W_T_prior` | zero | Target translation in `W` as an `{x, y, z}` object, in meters. |
| `rot_q_W_T_prior` | identity | Rotation from target frame `Ti` to `W` as an `{x, y, z, w}` quaternion. |

All configured targets must use the same fiducial type. The target list does not
use a `world_frame_flag`; world-frame selection is determined by `target_idx`.

## Priors and Coordinate Frames

Translations named `trans_A_B` express the origin of frame `B` in frame `A`.
Rotations named `rot_A_B` rotate vectors from `B` into `A`. Time offsets follow
`t_A = time_offset_A_B + t_B`. Quaternion JSON objects use the keys `x`, `y`,
`z`, and `w`.

The main frames are:

- `W`: world frame, defined by the target with the smallest `target_idx`.
- `B`: body frame, defined by the IMU with `body_frame_flag: true`.
- `Cb`: base camera frame, defined by the camera with `base_camera_flag: true`.
- `Ci`, `Ii`, `Mi`, `Ti`: the i-th camera, IMU, magnetometer, and target frames.

Priors are applied only when their corresponding `fix_*` option is enabled.
Parameter and frame definitions in the output JSON are also documented in
[calib_parameters.h](../include/xr_ucalib/uc_common/calib_parameter/calib_parameters.h).

## Generated Outputs

In compact invocation mode, final parameters are saved to
`<work_directory>/output_calib_params.json`. The file contains the estimated
camera intrinsics, IMU intrinsics, spatiotemporal extrinsics, target poses,
gravity direction, and magnetic-field direction when applicable.

Reusable and diagnostic artifacts are written below
`<work_directory>/ucalib_ws`, including cached camera detections, per-camera SfM
workspaces, camera-rig reprojection reports, and unified-calibration validation
results. These files can be large and are not required in a source-code commit.
Use the four-path command-line form when you want to place them elsewhere.

<p align="center">
	<img src="assets/logo.svg" alt="XR-UCalib" width="600"/>
</p>

<h2 align="center">
	A Unified Spatiotemporal Calibration Framework for Heterogeneous Multi-Sensor Systems in Extended Reality
</h2>

<p align="center">
	<img src="assets/calib_result.gif" alt="Calibration result demo" width="100%"/>
</p>

> **Note:** The paper associated with this project is currently under review. The full implementation (including complete source code, documentation, and datasets) will be made publicly available upon acceptance of the accompanying paper.

## 📖 Overview

XR-UCalib is a unified spatiotemporal calibration framework designed for immersive XR systems, delivering high-precision calibration to meet their stringent requirements. Key features include:

- **Unified Calibration:**
	Provides a unified framework for calibrating typical heterogeneous sensors in XR systems, avoiding the error accumulation of sequential processing. Covers global- and rolling-shutter cameras, IMUs, and magnetometers, supports arbitrary sensor counts, and is readily extensible to additional modalities.

- **Joint Spatiotemporal Estimation:**
	Simultaneous estimation of extrinsic parameters (rotation and translation) and temporal offsets within a single continuous-time optimization framework.

- **Flexible Low-Overlap Calibration:**
	Supports arbitrary numbers of calibration targets (e.g., AprilTag boards), enabling robust calibration for low-overlap and non-overlap camera rigs.

System flowchart is shown below.

![System flowchart](assets/system_flawchart.png)

## 📚 Table of Contents

- [📖 Overview](#-overview)
- [📚 Table of Contents](#-table-of-contents)
- [⚡ Quick Start](#-quick-start)
- [🛠️ Installation](#️-installation)
	- [Option A: Docker (Recommended)](#option-a-docker-recommended)
	- [Option B: Manual Environment Setup](#option-b-manual-environment-setup)
	- [Build XR-UCalib](#build-xr-ucalib)
- [🚀 Run the Project](#-run-the-project)
	- [Run Module Tests](#run-module-tests)
	- [Run with Example Data](#run-with-example-data)
	- [Run with Your Own Data](#run-with-your-own-data)
- [📝 Reference](#-reference)
- [📜 License](#-license)
- [🤝 Notes](#-notes)

---

## ⚡ Quick Start

Use the commands below for a minimal end-to-end run (build image, launch container, compile, and run the example dataset).

```bash
# 1) Clone and enter repository
git clone https://github.com/ylab-xrpg/xr-ucalib.git xr_ucalib
cd xr_ucalib

# 2) Build image
docker build -t ucalib_service:test .

# 3) Run container
docker run -it \
	--name run_ucalib \
	-e DISPLAY=$DISPLAY \
	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
	-v $(pwd):/workspace \
	-w /workspace \
	-p 9090:9090 \
	-p 9876:9876 \
	ucalib_service:test \
	/bin/bash

# 4) Build project
mkdir -p build && cd build
cmake .. && make -j"$(nproc)"

# 5) Run example
cd ../bin
./run_unified_calibration ../data/test_data_handheld
```

---

## 🛠️ Installation

XR-UCalib supports two installation paths.

> **Note:** The configuration and run instructions are based on Ubuntu.

### Option A: Docker (Recommended)

Clone the repository and build the image from the project root:

```bash
git clone https://github.com/ylab-xrpg/xr-ucalib.git xr_ucalib
cd xr_ucalib
docker build -t ucalib_service:test .
```

Run the container (with display forwarding, workspace mount, and visualization ports):

```bash
docker run -it \
	--name run_ucalib \
	-e DISPLAY=$DISPLAY \
	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
	-v $(pwd):/workspace \
	-w /workspace \
	-p 9090:9090 \
	-p 9876:9876 \
	ucalib_service:test \
	/bin/bash
```

### Option B: Manual Environment Setup

Clone the repository and enter the project directory first:

```bash
git clone https://github.com/ylab-xrpg/xr-ucalib.git xr_ucalib
cd xr_ucalib
```

First, install the required system dependencies:

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
	build-essential cmake ninja-build git curl ca-certificates unzip pkg-config \
	python3 python3-pip python3-dev python3-yaml python3-opencv \
	libgl1 libglib2.0-0 libx11-6 libxext6 libxrender1 libsm6 libxrandr2 libxfixes3 \
	libxi6 libxkbcommon0 libxcb1 \
	libatlas-base-dev libsuitesparse-dev libgoogle-glog-dev libgflags-dev \
	libeigen3-dev libopencv-dev \
	libboost-program-options-dev libboost-graph-dev libboost-system-dev libboost-filesystem-dev \
	libfreeimage-dev libmetis-dev libglew-dev qtbase5-dev libqt5opengl5-dev \
	libcgal-dev libflann-dev liblz4-dev libsqlite3-dev libjsoncpp-dev
```

Install required third-party libraries from source:

```bash
# Optional: use a dedicated local directory for source builds
mkdir -p ~/xr_ucalib_3rdparty && cd ~/xr_ucalib_3rdparty

# nlohmann_json v3.11.3
git clone --depth 1 --branch v3.11.3 https://github.com/nlohmann/json.git
cmake -S json -B json/build -G Ninja
cmake --build json/build -j"$(nproc)"
sudo cmake --install json/build

# spdlog v1.14.0
git clone --depth 1 --branch v1.14.0 https://github.com/gabime/spdlog.git
cmake -S spdlog -B spdlog/build -G Ninja
cmake --build spdlog/build -j"$(nproc)"
sudo cmake --install spdlog/build

# Sophus 1.22.10
git clone --depth 1 --branch 1.22.10 https://github.com/strasdat/Sophus.git
cmake -S Sophus -B Sophus/build -G Ninja
cmake --build Sophus/build -j"$(nproc)"
sudo cmake --install Sophus/build

# Ceres Solver 2.2.0
git clone --depth 1 --branch 2.2.0 https://github.com/ceres-solver/ceres-solver.git
cmake -S ceres-solver -B ceres-solver/build -G Ninja \
	-DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DMINIGLOG=OFF
cmake --build ceres-solver/build -j"$(nproc)"
sudo cmake --install ceres-solver/build

# GTest 1.13.0
git clone --depth 1 --branch v1.13.0 https://github.com/google/googletest.git
cmake -S googletest -B googletest/build -G Ninja -DBUILD_GMOCK=ON -DINSTALL_GTEST=ON
cmake --build googletest/build -j"$(nproc)"
sudo cmake --install googletest/build

# COLMAP (commit 9c704e89)
curl -fsSL https://github.com/colmap/colmap/archive/9c704e89.tar.gz -o colmap.tar.gz
mkdir -p colmap
tar -xzf colmap.tar.gz -C colmap --strip-components=1
# Note: You might encounter a "glog::glog" target conflict during cmake. The fix can be:
# sudo sed -i '1i if(TARGET glog::glog)\n  return()\nendif()\n' /usr/local/lib/cmake/Ceres/FindGlog.cmake
cmake -S colmap -B colmap/build -G Ninja \
	-DCMAKE_BUILD_TYPE=Release -DGUI_ENABLED=OFF -DCUDA_ENABLED=OFF -DTESTS_ENABLED=OFF \
	-DGLOG_PREFER_EXPORTED_GLOG_CMAKE_CONFIGURATION=ON
cmake --build colmap/build -j"$(nproc)"
sudo cmake --install colmap/build
rm -f colmap.tar.gz
```

Optional: install Python dependencies for visualization tools only (calibration core does not require them).
If you need `rerun-sdk`, Python 3.10+ is recommended:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install numpy scipy matplotlib rerun-sdk pyx pyyaml opencv-python-headless
```

### Build XR-UCalib

After environment setup (both Docker and manual use the same build flow):

```bash
cd xr_ucalib
mkdir -p build
cd build
cmake ..
make -j"$(nproc)"
```

---

## 🚀 Run the Project

### Run Module Tests

```bash
cd bin
./run_tests
```

### Run with Example Data

```bash
cd bin
./run_unified_calibration ../data/test_data_handheld
```

> To reduce repository size, the example dataset does not include original camera images.
> Only feature detections are provided in [data/test_data_handheld/ucalib_ws/sensor_manager/cam0_detections.json](data/test_data_handheld/ucalib_ws/sensor_manager/cam0_detections.json).

More benchmark datasets are under preparation and will be released in future updates.

### Run with Your Own Data

You can organize your data folder by following the provided sample structure:

```text
your_dataset/
├── sensor_data
│   ├── cam0
│   ├── cam1
│   ├── cam2
│   ├── cam3
│   ├── imu0.csv
│   ├── imu1.csv
│   └── mag1.csv
└── input_config.json
```

Data conventions:

- All sensors are assumed to be driven by the same clock source, with only small residual time offsets (typically millisecond-level).
- Camera image filenames in each `camX` directory must be nanosecond timestamps.
- IMU CSV format must be:
	`timestamp(ns),gyr_x(rad/s),gyr_y(rad/s),gyr_z(rad/s),acc_x(m/s^2),acc_y(m/s^2),acc_z(m/s^2)`
- Magnetometer CSV format must be:
	`timestamp(ns),mag_x,mag_y,mag_z`
- Magnetometer input is expected to be a normalized vector after intrinsic correction (e.g., ellipsoid-fitting calibration).

The calibrator behavior is fully configured by [input_config.json](data/test_data_handheld/input_config.json).
Detailed parameter descriptions are available in [system_config.h](include/xr_ucalib/uc_common/config/system_config.h).

Calibration outputs are saved to [output_calib_params.json](data/test_data_handheld/output_calib_params.json) in your dataset folder (for example, `data/test_data_handheld/output_calib_params.json`).
Estimated parameters include spatiotemporal extrinsics, camera intrinsics, and IMU intrinsic parameters.
Parameter and coordinate-frame definitions can be found in [calib_parameters.h](include/xr_ucalib/uc_common/calib_parameter/calib_parameters.h).

---

## 📝 Reference

If you use this project in your research, please cite:

```bibtex
@article{shu2026xrucalib,
  title   = {XR-UCalib: A Unified Spatiotemporal Calibration Framework
             for Heterogeneous Multi-Sensor XR Systems},
  author  = {Zichao Shu and Lijun Li and Xudong Zhang and Hongjun Fang and
             Xinlei Chu and Zetao Chen and Jianyu Wang},
  journal = {Under Review},
  year    = {2026},
}
```

## 📜 License

Copyright 2026 Yongjiang Laboratory

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

---

## 🤝 Notes

- We welcome feedback, issues, and contributions.
- If you find this project useful, please consider citing it once the paper/release metadata is publicly available.
- The full implementation, including complete source code and datasets, will be released upon acceptance of the accompanying paper.

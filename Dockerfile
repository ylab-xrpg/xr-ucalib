FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

# Use a faster mirror.
# RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|http://mirrors.aliyun.com/ubuntu/|g' /etc/apt/sources.list

# Core build/runtime dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \
	build-essential \
	cmake \
	ninja-build \
	git \
	curl \
	ca-certificates \
	unzip \
	pkg-config \
	python3 \
	python3-pip \
	python3-dev \
	python3-yaml \
	python3-opencv \
	libgl1 \
	libglib2.0-0 \
	libx11-6 \
	libxext6 \
	libxrender1 \
	libsm6 \
	libxrandr2 \
	libxfixes3 \
	libxi6 \
	libxkbcommon0 \
	libxcb1 \
	libatlas-base-dev \
	libsuitesparse-dev \
	libgoogle-glog-dev \
	libgflags-dev \
	libeigen3-dev \
	libopencv-dev \
	libboost-program-options-dev \
	libboost-graph-dev \
	libboost-system-dev \
	libboost-filesystem-dev \
	libfreeimage-dev \
	libmetis-dev \
	libglew-dev \
	qtbase5-dev \
	libqt5opengl5-dev \
	libcgal-dev \
	libflann-dev \
	liblz4-dev \
	libsqlite3-dev \
	libjsoncpp-dev

# Install nlohmann_json 3.11.3 from source.
RUN git clone --depth 1 --branch v3.11.3 https://github.com/nlohmann/json.git /tmp/json && \
	cmake -S /tmp/json -B /tmp/json/build -G Ninja && \
	cmake --build /tmp/json/build -j"$(nproc)" && \
	cmake --install /tmp/json/build && \
	rm -rf /tmp/json

# Install spdlog 1.14.0 from source.
RUN git clone --depth 1 --branch v1.14.0 https://github.com/gabime/spdlog.git /tmp/spdlog && \
	cmake -S /tmp/spdlog -B /tmp/spdlog/build -G Ninja && \
	cmake --build /tmp/spdlog/build -j"$(nproc)" && \
	cmake --install /tmp/spdlog/build && \
	rm -rf /tmp/spdlog

# Install Sophus 1.22.10 from source.
RUN git clone --depth 1 --branch 1.22.10 https://github.com/strasdat/Sophus.git /tmp/Sophus && \
	cmake -S /tmp/Sophus -B /tmp/Sophus/build -G Ninja && \
	cmake --build /tmp/Sophus/build -j"$(nproc)" && \
	cmake --install /tmp/Sophus/build && \
	rm -rf /tmp/Sophus

# Install Ceres Solver 2.2.0 from source (required by COLMAP).
RUN git clone --depth 1 --branch 2.2.0 https://github.com/ceres-solver/ceres-solver.git /tmp/ceres-solver && \
	cmake -S /tmp/ceres-solver -B /tmp/ceres-solver/build -G Ninja \
	  -DBUILD_TESTING=OFF \
	  -DBUILD_EXAMPLES=OFF \
	  -DMINIGLOG=OFF && \
	cmake --build /tmp/ceres-solver/build -j"$(nproc)" && \
	cmake --install /tmp/ceres-solver/build && \
	rm -rf /tmp/ceres-solver

# Patch Ceres's bundled FindGlog.cmake to guard against duplicate target creation.
# Without this, COLMAP's cmake finds glog first (creating glog::glog), then
# find_package(Ceres) internally re-runs FindGlog.cmake which tries to create
# the same imported target again, causing a fatal error.
RUN sed -i '1i if(TARGET glog::glog)\n  return()\nendif()\n' \
	/usr/local/lib/cmake/Ceres/FindGlog.cmake

# Install GTest 1.13.0 from source.
RUN git clone --depth 1 --branch v1.13.0 https://github.com/google/googletest.git /tmp/googletest && \
	cmake -S /tmp/googletest -B /tmp/googletest/build -G Ninja \
	  -DBUILD_GMOCK=ON \
	  -DINSTALL_GTEST=ON && \
	cmake --build /tmp/googletest/build -j"$(nproc)" && \
	cmake --install /tmp/googletest/build && \
	rm -rf /tmp/googletest

# Install COLMAP from source (commit 9c704e89).
# Use tarball download instead of git clone to avoid GnuTLS network errors.
RUN curl -fsSL https://github.com/colmap/colmap/archive/9c704e89.tar.gz \
	  -o /tmp/colmap.tar.gz && \
	mkdir -p /tmp/colmap && \
	tar -xzf /tmp/colmap.tar.gz -C /tmp/colmap --strip-components=1 && \
	rm /tmp/colmap.tar.gz && \
	cmake -S /tmp/colmap -B /tmp/colmap/build -G Ninja \
	  -DCMAKE_BUILD_TYPE=Release \
	  -DGUI_ENABLED=OFF \
	  -DCUDA_ENABLED=OFF \
	  -DTESTS_ENABLED=OFF \
	  -DGLOG_PREFER_EXPORTED_GLOG_CMAKE_CONFIGURATION=ON && \
	cmake --build /tmp/colmap/build -j"$(nproc)" && \
	cmake --install /tmp/colmap/build && \
	rm -rf /tmp/colmap

# Install Python 3.10 via Miniconda (needed by rerun-sdk which requires >= 3.9).
# Fetched from repo.anaconda.com to avoid GitHub/PPA network issues.
RUN curl -fsSL https://repo.anaconda.com/miniconda/Miniconda3-py310_24.1.2-0-Linux-x86_64.sh \
	  -o /tmp/miniconda.sh && \
	bash /tmp/miniconda.sh -b -p /opt/conda && \
	rm /tmp/miniconda.sh && \
	ln -sf /opt/conda/bin/python3.10 /usr/local/bin/python3.10 && \
	ln -sf /opt/conda/bin/python3.10 /usr/local/bin/python3 && \
	ln -sf /opt/conda/bin/python3.10 /usr/local/bin/python && \
	ln -sf /opt/conda/bin/pip /usr/local/bin/pip3 && \
	ln -sf /opt/conda/bin/pip /usr/local/bin/pip

# Python dependencies used by scripts (installed under Python 3.10 via conda).
# Use Tsinghua PyPI mirror to improve download reliability.
RUN /opt/conda/bin/pip install --no-cache-dir \
	-i https://pypi.tuna.tsinghua.edu.cn/simple \
	numpy \
	scipy \
	matplotlib \
	rerun-sdk \
	pyx \
	pyyaml \
	opencv-python-headless

# Final cleanup.
RUN apt-get autoremove -y && \
	apt-get clean && \
	rm -rf /var/lib/apt/lists/*

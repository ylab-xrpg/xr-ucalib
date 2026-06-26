cmake_minimum_required(VERSION 3.11)
project(XRUCalib VERSION 0.1.0)

# Set default build type to Debug if not specified
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# C++ standard settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler options for MSVC and GCC/Clang
if(MSVC)
  add_compile_options(/W3 /O2 /bigobj)  # /bigobj: allow larger object files
else()
  add_compile_options(-Wall -O3)
endif()

# Handle Conda environment
if(DEFINED ENV{CONDA_PREFIX})
  message(STATUS "Conda environment detected: $ENV{CONDA_PREFIX}")
  # 1. Prioritize system paths (/usr/local, /usr) over Conda paths.
  # This ensures that find_package(), find_library(), etc. look in system dirs first.
  list(INSERT CMAKE_PREFIX_PATH 0 "/usr/local" "/usr")
  
  # 2. Add Conda prefix to ignore path to prevent picking up its libraries/headers
  # if they are not explicitly requested.
  list(APPEND CMAKE_IGNORE_PATH "$ENV{CONDA_PREFIX}")
  
  # 3. For some versions of CMake, CMAKE_SYSTEM_IGNORE_PATH is also needed.
  list(APPEND CMAKE_SYSTEM_IGNORE_PATH "$ENV{CONDA_PREFIX}")
endif()

# Third-party dependencies
# Nlohmann JSON
find_package(nlohmann_json 3.11.3 QUIET)
if(NOT nlohmann_json_FOUND)
  message(FATAL_ERROR "nlohmann_json not found")
else()
  if(NOT nlohmann_json_VERSION VERSION_EQUAL "3.11.3")
    message(WARNING "Found nlohmann_json ${nlohmann_json_VERSION}, recommended version is 3.11.3")
  endif()
  message(STATUS "nlohmann_json version: ${nlohmann_json_VERSION}")
endif()

# spdlog
find_package(spdlog 1.14.0 QUIET)
if(NOT spdlog_FOUND)
  message(FATAL_ERROR "spdlog not found")
else()
  if(NOT spdlog_VERSION VERSION_EQUAL "1.14.0")
    message(WARNING "Found spdlog ${spdlog_VERSION}, recommended version is 1.14.0")
  endif()
  message(STATUS "spdlog version: ${spdlog_VERSION}")
endif()

# Eigen
find_package(Eigen3 3.3.7 QUIET)
if(NOT Eigen3_FOUND)
  message(FATAL_ERROR "Eigen3 not found")
else()
  if(NOT Eigen3_VERSION VERSION_EQUAL "3.3.7")
    message(WARNING "Found Eigen3 ${Eigen3_VERSION}, recommended version is 3.3.7")
  endif()
  message(STATUS "Eigen3 version: ${Eigen3_VERSION}")
endif()

# Sophus
find_package(Sophus 1.22.10 QUIET)
if(NOT Sophus_FOUND)
  message(FATAL_ERROR "Sophus not found")
else()
  if(NOT Sophus_VERSION VERSION_EQUAL "1.22.10")
    message(WARNING "Found Sophus ${Sophus_VERSION}, recommended version is 1.22.10")
  endif()
  message(STATUS "Sophus version: ${Sophus_VERSION}")
endif()

# Ceres (found via COLMAP)
# NOTE: Ceres is a dependency of COLMAP. Finding it explicitly here causes a conflict
# because Ceres's FindGlog.cmake attempts to define the 'glog::glog' target twice
# (once here, and once inside find_package(COLMAP)).
# We let COLMAP find Ceres for us.
# find_package(Ceres 2.2.0 QUIET)
# if(NOT Ceres_FOUND)
#   message(FATAL_ERROR "Ceres not found")
# else()
#   if(NOT Ceres_VERSION VERSION_EQUAL "2.2.0")
#     message(WARNING "Found Ceres ${Ceres_VERSION}, recommended version is 2.2.0")
#   endif()
#   message(STATUS "Ceres version: ${Ceres_VERSION}")
# endif()

# OpenCV
# Most OpenCV versions (3.4.x, 4.x, etc.) are compatible.
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV not found")
else()
  message(STATUS "OpenCV version: ${OpenCV_VERSION}")
endif()

# GTest
find_package(GTest 1.13.0 QUIET)
if(NOT GTest_FOUND)
  message(FATAL_ERROR "GTest not found")
else()
  if(NOT GTest_VERSION VERSION_EQUAL "1.13.0")
    message(WARNING "Found GTest ${GTest_VERSION}, recommended version is 1.13.0")
  endif()
  message(STATUS "GTest version: ${GTest_VERSION}")
endif()

# COLMAP
find_package(COLMAP 3.9 QUIET)
if(NOT COLMAP_FOUND)
  message(FATAL_ERROR "COLMAP not found")
else()
  if(NOT COLMAP_VERSION VERSION_EQUAL "3.9")
    message(WARNING "Found COLMAP ${COLMAP_VERSION}, recommended version is 3.9")
  endif()
  message(STATUS "COLMAP version: ${COLMAP_VERSION}")
  
  # Verify Ceres was found by COLMAP
  if(Ceres_FOUND)
     message(STATUS "Ceres version (found via COLMAP): ${Ceres_VERSION}")
  endif()
endif()

# OpenMP
find_package(OpenMP REQUIRED)

# Project modules
add_subdirectory(third_party/ethz_apriltag2)  # AprilTag2

add_subdirectory(src/uc_common)               # Common utilities

add_subdirectory(src/uc_cam_calib)            # Camera calibration library

add_subdirectory(src/uc_unified_calib)        # Unified calibration library

add_subdirectory(tests)                       # Unit tests

add_subdirectory(app)                         # Executable applications

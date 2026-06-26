# Collect all source files under the current directory recursively.
file(GLOB_RECURSE UC_CAM_CALIB_SOURCES "*.cpp" "*.cc")

# Create a static library from the collected sources.
add_library(uc_cam_calib STATIC ${UC_CAM_CALIB_SOURCES})

# Specify include directories for uc_cam_calib.
target_include_directories(uc_cam_calib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIR}
)

# Link dependencies required by uc_cam_calib.
target_link_libraries(uc_cam_calib
  PUBLIC
    uc_common
    Ceres::ceres
    colmap::colmap
    OpenMP::OpenMP_CXX
    Sophus::Sophus
    spdlog::spdlog_header_only
    ${OpenCV_LIBS}
)

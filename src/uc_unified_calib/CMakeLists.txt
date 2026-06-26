# Collect all source files under the current directory recursively.
file(GLOB_RECURSE UC_UNIFIED_SOURCES "*.cpp" "*.cc")

# Create a static library from the collected sources.
add_library(uc_unified_calib STATIC ${UC_UNIFIED_SOURCES})

# Specify include directories for uc_unified_calib.
target_include_directories(uc_unified_calib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIR}
)

# Link dependencies required by uc_unified_calib.
target_link_libraries(uc_unified_calib
  PUBLIC
    Ceres::ceres
    nlohmann_json::nlohmann_json
    Sophus::Sophus
    spdlog::spdlog_header_only
    ${OpenCV_LIBS}
)

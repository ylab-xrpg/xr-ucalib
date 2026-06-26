# Collect all source files under the current directory recursively.
file(GLOB_RECURSE UC_COMMON_SOURCES "*.cpp" "*.cc")

# Create a static library from the collected sources.
add_library(uc_common STATIC ${UC_COMMON_SOURCES})

# Specify include directories for uc_common.
target_include_directories(uc_common
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIR}
)

# Link dependencies required by uc_common.
target_link_libraries(uc_common
  PUBLIC
    apriltag
    Ceres::ceres
    nlohmann_json::nlohmann_json
    Sophus::Sophus
    spdlog::spdlog_header_only
    ${OpenCV_LIBS}
)

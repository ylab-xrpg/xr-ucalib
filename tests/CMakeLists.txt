# Collect test sources and reconfigure automatically when a test is added.
file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
)

# Tests read the tracked inputs but write generated configuration, calibration
# output, COLMAP databases, and workspace artifacts only under /tmp.
set(XR_UCALIB_TEST_OUTPUT_DIR "/tmp/xr_ucalib_tests")
file(MAKE_DIRECTORY "${XR_UCALIB_TEST_OUTPUT_DIR}/ucalib_ws")
file(COPY
  "${PROJECT_SOURCE_DIR}/data/test_data_handheld/ucalib_ws/sensor_manager"
  DESTINATION "${XR_UCALIB_TEST_OUTPUT_DIR}/ucalib_ws"
)

# Create the test executable.
add_executable(run_tests ${TEST_SOURCES})

target_compile_definitions(run_tests
  PRIVATE
    XR_UCALIB_TEST_DATA_DIR="${PROJECT_SOURCE_DIR}/data/test_data_handheld"
    XR_UCALIB_TEST_OUTPUT_DIR="${XR_UCALIB_TEST_OUTPUT_DIR}"
)

# Link test executable with required libraries.
target_link_libraries(run_tests
  PRIVATE
    uc_common
    uc_cam_calib
    uc_unified_calib
    GTest::gtest
)

include(GoogleTest)
gtest_discover_tests(run_tests)

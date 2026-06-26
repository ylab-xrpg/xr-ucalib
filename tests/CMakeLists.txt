# Collect all test source files (not used directly here but can be used for future expansion).
file(GLOB_RECURSE TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cc")

# Create the test executable.
add_executable(run_tests ${TEST_SOURCES})

# Link test executable with required libraries.
target_link_libraries(run_tests
  PRIVATE
    uc_common
    uc_cam_calib
    uc_unified_calib
    GTest::gtest
    GTest::gtest_main
)

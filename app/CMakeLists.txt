# Create the executable.
add_executable(run_unified_calibration run_unified_calibration.cc)

# Link the executable with the library.
target_link_libraries(run_unified_calibration
  PRIVATE
    uc_common
    uc_cam_calib
    uc_unified_calib
)
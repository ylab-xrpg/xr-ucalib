# Collect all source files under the current directory recursively.
file(GLOB_RECURSE APRILTAG_SOURCES "*.cpp" "*.cc")

# Find and enable OpenMP
find_package(OpenMP REQUIRED)

# Create a static library from the collected sources.
add_library(apriltag STATIC ${APRILTAG_SOURCES})

# Specify include directories for uc_apriltag.
target_include_directories(apriltag
  PUBLIC
    include/
    ${EIGEN3_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIR}
)

# Link dependencies required by uc_apriltag.
target_link_libraries(apriltag 
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
)

cmake_minimum_required(VERSION 3.8)
project(common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# Add include directories
include_directories(include/common
                    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>)

# Add common shared library
# https://www.theconstructsim.com/how-to-create-a-ros2-c-library/
# https://github.com/ros2/eigen3_cmake_module
# -
add_library(common src/common.cpp)
ament_target_dependencies(common Eigen3)

# Build and export library
ament_export_targets(common HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

install(DIRECTORY include/common DESTINATION include)

install(
  TARGETS common
  EXPORT common
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

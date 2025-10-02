cmake_minimum_required(VERSION 3.8)
project(combined_test)

# Compiler warnings (optional, since we aren’t compiling any nodes here)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required build system
find_package(ament_cmake REQUIRED)

# This package only needs to install files (no executables)
install(
  DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE
)

ament_package()

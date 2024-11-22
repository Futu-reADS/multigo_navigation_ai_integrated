cmake_minimum_required(VERSION 3.5)
project(boot)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# List of dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclpy
)

# If you don’t need any executables in this package, skip executable targets
# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config directory into the share folder
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ament package macros
ament_package()


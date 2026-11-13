cmake_minimum_required(VERSION 3.8)
project(yam_camera)

find_package(ament_cmake REQUIRED)

# Install the launch files and the camera settings with the package.
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

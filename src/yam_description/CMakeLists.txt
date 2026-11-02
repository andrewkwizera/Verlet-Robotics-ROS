cmake_minimum_required(VERSION 3.8)
project(yam_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY
    urdf
    meshes
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
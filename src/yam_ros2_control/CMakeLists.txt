cmake_minimum_required(VERSION 3.8)
project(yam_ros2_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Physical YAM CAN communication library
add_library(
  yam_can_driver
  SHARED
  src/yam_can_driver.cpp
)

target_compile_features(
  yam_can_driver
  PUBLIC cxx_std_17
)

target_include_directories(
  yam_can_driver
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ros2_control hardware plugin (mock or real, chosen in the URDF)
add_library(
  yam_hardware_interface
  SHARED
  src/yam_hardware_interface.cpp
)

target_compile_features(
  yam_hardware_interface
  PUBLIC cxx_std_17
)

target_include_directories(
  yam_hardware_interface
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# The plugin uses the CAN driver to talk to real motors.
target_link_libraries(
  yam_hardware_interface
  yam_can_driver
)

ament_target_dependencies(
  yam_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  hardware_interface
  yam_ros2_control.xml
)

install(
  TARGETS yam_can_driver yam_hardware_interface
  EXPORT export_yam_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(
  export_yam_hardware_interface HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Physical YAM CAN communication library
add_library(
  yam_can_driver
  SHARED
  src/yam_can_driver.cpp
)

target_compile_features(
  yam_can_driver
  PUBLIC cxx_std_17
)

target_include_directories(
  yam_can_driver
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS yam_can_driver
  LIBRARY DESTINATION lib
)

add_library(
  yam_physical_hardware
  SHARED
  src/yam_physical_hardware.cpp
)

target_compile_features(
  yam_physical_hardware
  PUBLIC cxx_std_17
)

target_include_directories(
  yam_physical_hardware
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  yam_physical_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(
  yam_physical_hardware
  yam_can_driver
)

install(
  TARGETS yam_physical_hardware
  LIBRARY DESTINATION lib
)

ament_package()

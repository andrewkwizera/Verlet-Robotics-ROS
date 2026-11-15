cmake_minimum_required(VERSION 3.8)
project(yam_sim_display)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

# --- Operator dashboard (Milestone 1) ---------------------------------------
# Installed entry point: resolved from lib/${PROJECT_NAME} so `ros2 run` and
# `ros2 launch` find it on the normal executable path.
install(
  PROGRAMS dashboard/run.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dashboard_node
)

# Importable package (telemetry, config, HTTP transport, node) plus the static
# web assets. Installed under share/ so runtime lookups never depend on the
# source tree.
install(
  DIRECTORY dashboard/
  DESTINATION share/${PROJECT_NAME}/dashboard
  FILES_MATCHING
    PATTERN "*.py"
    PATTERN "*.html"
    PATTERN "*.css"
    PATTERN "*.js"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright/cpplint checks: launch-only package with no C++ sources.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

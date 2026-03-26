cmake_minimum_required(VERSION 3.8)
project(nav)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PkgConfig)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

set(dependencies
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

include_directories(include)

# install all source/config/data directories into the package share
install(
  DIRECTORY launch config waypoints nav src scripts
  DESTINATION share/${PROJECT_NAME}
)

# ── C++ executables (unchanged) ──────────────────────────────────── #
add_executable(pure_pursuit src/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit ${dependencies})

add_executable(smoothing src/smoothing.cpp)
ament_target_dependencies(smoothing ${dependencies})

install(TARGETS
  pure_pursuit
  smoothing
  DESTINATION lib/${PROJECT_NAME}
)

# ── Python executables (new) ─────────────────────────────────────── #
# Each gets its own install() so we can RENAME .py → no extension
# This lets "ros2 run nav mpc_tracker" work correctly
install(PROGRAMS scripts/mpc_tracker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mpc_tracker)

install(PROGRAMS scripts/spawn_static_obstacle.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME spawn_static_obstacle)

install(PROGRAMS scripts/spawn_dynamic_obstacle.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME spawn_dynamic_obstacle)

ament_package()

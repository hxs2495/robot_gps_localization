cmake_minimum_required(VERSION 3.5)
project(robot_odom_transform)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Ubuntu's GeographicLib package installs its Find module here on ROS 2 Humble.
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

add_executable(gpgga_filter_node src/gpgga_filter_node.cpp)
ament_target_dependencies(gpgga_filter_node rclcpp sensor_msgs)

add_executable(gps_to_utm_odometry_node src/gps_to_utm_odometry_node.cpp)
ament_target_dependencies(
  gps_to_utm_odometry_node
  GeographicLib
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
)

add_executable(gps_global_odometry_node src/gps_global_odometry_node.cpp)
ament_target_dependencies(
  gps_global_odometry_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(odometry_tf_transform_node src/odometry_tf_transform_node.cpp)
ament_target_dependencies(
  odometry_tf_transform_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(
  TARGETS
    gpgga_filter_node
    gps_to_utm_odometry_node
    gps_global_odometry_node
    odometry_tf_transform_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The package has not declared a license/owner yet; do not invent legal metadata.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

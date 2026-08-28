cmake_minimum_required(VERSION 3.5)
project(robot_odom_transform)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(geodesy REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(nmea_msgs REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# 李佳宾 2025/8/11 去除多余节点
add_executable(test_odometry_transformstamped src/test_odometry_transformstamped.cpp)
add_executable(gpgga_filter_node src/gpgga_filter_node.cpp)
add_executable(gpgga_kalman_filter_node src/gpgga_kalman_filter_node.cpp)
add_executable(tf_odom_to_gps src/tf_odom_to_gps.cpp)
add_executable(gps_to_utm_odometry_node src/gps_to_utm_odometry_node.cpp)
add_executable(utm_to_gps_odometry_node src/utm_to_gps_odometry_node.cpp)
add_executable(gps_global_odometry_node src/gps_global_odometry_node.cpp)

# jiabin add
add_executable(odom_to_gps_node src/odom_to_gps_node.cpp)
add_executable(tf_init_node src/tf_init_node.cpp)
add_executable(odom_repub_with_tf src/odom_repub_with_tf.cpp)

# huangxiaoshuai add
ament_target_dependencies(
  gpgga_filter_node
  rclcpp
  sensor_msgs
  nmea_msgs
)
# huangxiaoshuai add
ament_target_dependencies(
  gpgga_kalman_filter_node
  rclcpp
  sensor_msgs
  nmea_msgs
)


# huangxiaoshuai add
ament_target_dependencies(tf_odom_to_gps
rclcpp
sensor_msgs
nav_msgs
tf2
tf2_ros
tf2_geometry_msgs
geometry_msgs
)

# huangxiaoshuai add
ament_target_dependencies(test_odometry_transformstamped
rclcpp
sensor_msgs
nav_msgs
tf2
tf2_ros
tf2_geometry_msgs
geometry_msgs
)

# jiabin add
ament_target_dependencies(odom_to_gps_node
rclcpp
sensor_msgs
nav_msgs
tf2
tf2_ros
tf2_geometry_msgs
geometry_msgs
GeographicLib
robot_interfaces
)

# jiabin add 25/7/18
ament_target_dependencies(tf_init_node
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

# jiabin add 25/7/21
ament_target_dependencies(odom_repub_with_tf 
  rclcpp
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  )

# jiabin add 9/23
add_executable(ex_save_node src/ex_save_node.cpp)

ament_target_dependencies(ex_save_node
  rclcpp
  geometry_msgs
  std_msgs
)
# huangxiaoshuai add
ament_target_dependencies(gps_to_utm_odometry_node rclcpp sensor_msgs tf2_geometry_msgs nav_msgs GeographicLib robot_interfaces)
ament_target_dependencies(utm_to_gps_odometry_node rclcpp sensor_msgs nav_msgs GeographicLib robot_interfaces)
ament_target_dependencies(gps_global_odometry_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(
  TARGETS
  # 李佳宾 2025/8/11 去除多余节点
  utm_to_gps_odometry_node # huangxiaoshuai add
  test_odometry_transformstamped # huangxiaoshuai add
  gpgga_filter_node  # huangxiaoshuai add
  tf_odom_to_gps # huangxiaoshuai add
  gps_to_utm_odometry_node # huangxiaoshuai add
  odom_to_gps_node  # jiabin add
  tf_init_node  # jiabin add 25/7/18
  odom_repub_with_tf # jiabin add 25/7/21
  ex_save_node # jiabin add 9/23
  gpgga_kalman_filter_node # huangxiaoshuai add
  gps_global_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch和默认参数
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
ament_package()

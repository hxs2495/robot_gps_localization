cmake_minimum_required(VERSION 3.8)
project(robot_bag_play_tool)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

set(CMAKE_AUTOMOC ON)

include_directories(include)

add_executable(robot_bag_play_tool
  src/main.cpp
  src/main_window.cpp
  src/bag_player.cpp
  src/message_parser.cpp
  src/timeline_widget.cpp
  include/robot_bag_play_tool/main_window.hpp
  include/robot_bag_play_tool/bag_player.hpp
  include/robot_bag_play_tool/message_parser.hpp
  include/robot_bag_play_tool/timeline_widget.hpp
)

ament_target_dependencies(robot_bag_play_tool
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rosgraph_msgs
  rosidl_typesupport_introspection_cpp
)

target_link_libraries(robot_bag_play_tool
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
)

install(TARGETS robot_bag_play_tool
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()

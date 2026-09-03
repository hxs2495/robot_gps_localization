cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

# Keep config/urdf/robot.urdf as the repository's single editable model and
# install that exact file into the ROS package share directory.
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../config/urdf/robot.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()

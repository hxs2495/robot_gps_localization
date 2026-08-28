cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  


  # 模块1：机器人状态监控模块相关接口
  "msg/Gnss.msg"
  "msg/ZoneNorthp.msg"
  "msg/Stage.msg"

  # 模块2：任务管理与调度模块

  
  # 模块3：数据采集与处理模块 均为ros2标准消息接口数据类型

  # 模块4：运行系统监控模块相关接口
    "msg/CPUUsage.msg"
    "msg/MemoryUsage.msg"
    "msg/HardwareHealth.msg"
  
  # 模块5：地图加载轨迹显示模块相关接口

  # # 日志管理服务接口
  # "srv/GetLogDates.srv"
  # "srv/GetLogNodes.srv"
  # "srv/QueryLogs.srv"
  # "srv/DeleteLogs.srv"
  # "srv/DownloadLogs.srv"
  # "srv/GetLogContent.srv"

  
  DEPENDENCIES builtin_interfaces std_msgs nav2_msgs geometry_msgs
)

ament_package()

# robot_ekf_localization

该包将所有传感器观测先转换到同一个 `base_footprint`，再执行局部、全局双 EKF。
滤波器不再假设 FAST-LIO 原点、GPS 天线和车体原点重合。

## 融合链路

```text
/Odometry (odom -> livox_imu)
  -> TF参考点转换 -> /odometry/lio/base (odom -> base_footprint)
  -> 局部EKF -> /odometry/local
                                      \
                                       -> 全局EKF -> /odometry/global
                                      /
/odometry/gps (map -> base_footprint)
  -> GPS失效/恢复平滑 -> /odometry/gps/smoothed
```

- FAST-LIO 内部状态位于 `livox_imu`，其 LiDAR-IMU 初始外参从 TF 获取；
- 局部 EKF 只消费已转换的 `/odometry/lio/base`；
- GPS 节点先按 TF 把天线轨迹换算成 `base_footprint` 轨迹；
- 完整融合时全局 EKF 是唯一的动态 `odom -> base_footprint` 发布者；
- 当前方案使用单位静态 `map -> odom`，两套里程计都在同一局部原点下比较。

## 推荐启动

rosbag 回放：

```bash
source install/setup.bash
ros2 launch robot_ekf_localization localization_replay.launch.py \
  bag_path:=robot_data/all-data-8-23-4
```

该入口默认使用 `ROS_DOMAIN_ID=42` 并调用标准 rosbag2 播放器，避免 Gazebo、
Livox 实时驱动或其他回放工具形成多个 `/clock`/传感器发布者。

实时传感器：

```bash
ros2 launch robot_ekf_localization localization_bringup.launch.py \
  use_sim_time:=false
```

统一入口只启动一个 `robot_state_publisher`，依次纳入机器人模型、FAST-LIO、GPS
转换和双 EKF。传感器安装改变时无需修改融合 YAML 或 C++。

仍可独立启动局部或完整 EKF：

```bash
ros2 launch robot_ekf_localization local_ekf_localization.launch.py
ros2 launch robot_ekf_localization gps_localization.launch.py
```

主要接口：

```text
lio_sensor_odom_topic:=/Odometry
lio_base_odom_topic:=/odometry/lio/base
lio_sensor_frame:=livox_imu
base_frame:=base_footprint
gps_odom_topic:=/odometry/gps
```

## TF 所有权

```text
robot_state_publisher : base_footprint -> base_link -> {livox_imu, gps}
                                              livox_imu -> livox_frame
static publisher      : map -> odom
global EKF            : odom -> base_footprint
```

不要再运行 GPS、FAST-LIO 或其他节点发布上述同名动态 TF。安装外参只修改
`config/urdf/robot.urdf`。

## 验证

```bash
ros2 topic echo /Odometry --once
ros2 topic echo /odometry/lio/base --once
ros2 topic echo /odometry/gps --once
ros2 topic hz /odometry/global

ros2 run tf2_ros tf2_echo base_footprint gps
ros2 run tf2_ros tf2_echo livox_imu livox_frame
ros2 run tf2_ros tf2_echo odom base_footprint
```

应看到 `/Odometry.child_frame_id=livox_imu`，而送入滤波器的激光和 GPS 里程计
都应为 `child_frame_id=base_footprint`。

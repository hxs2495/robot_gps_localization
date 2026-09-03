# Robot GPS Localization

基于 FAST-LIO、GNSS 和 `robot_localization` 的 ROS 2 融合定位项目。当前架构将
URDF/TF 作为全部静态传感器安装外参的唯一来源，传感器换位置后不需要修改算法
代码或在多个 YAML 中同步外参。

## 为什么必须先统一参考点

GPS 天线、LiDAR/IMU 和车体旋转中心不在同一点时，它们在转弯中的原始轨迹本来
就不会重合。不能通过给两条轨迹增加一个固定平移来解决，因为杆臂随车体姿态一起
旋转。正确关系是完整的 SE(3) 刚体变换：

```text
T_world_base = T_world_sensor * T_sensor_base
```

本项目保留每个算法的原生传感器参考点，然后通过 TF 把 GPS 和激光里程计都转换
成 `base_footprint` 的位姿后再融合。这样转弯引起的杆臂圆弧运动会被正确消除。

## 坐标与数据架构

```text
                            config/urdf/robot.urdf
                                      |
                           robot_state_publisher
                                      |
          +---------------------------+--------------------------+
          |                                                      |
LiDAR + IMU -> FAST-LIO -- /Odometry (odom -> livox_imu)         |
          |                 + TF livox_imu -> base_footprint      |
          +--------------> /odometry/lio/base ----------------+   |
                                                             |   |
GNSS /fix + heading -> /utm/gps (utm -> gps)                 |   |
          + TF gps -> base_footprint                          |   |
          +--------------> /odometry/gps ----------------+    |   |
                                                         v    v   |
                                                    local/global EKF
                                                         |
                                                /odometry/global
```

静态 TF 树：

```text
base_footprint
└── base_link
    ├── livox_imu
    │   └── livox_frame
    └── gps
```

- `base_footprint`：所有融合输入和最终定位统一使用的车体参考点；
- `livox_imu`：FAST-LIO 状态原点和原生 `/Odometry` 的 child frame；
- `livox_frame`：点云坐标系；FAST-LIO 启动时从 TF 读取
  `T_livox_imu_livox_frame`；
- `gps`：天线/航向观测坐标系；GPS 节点从 TF 读取 `T_gps_base_footprint`。

## 外参唯一配置

唯一可编辑模型是 [config/urdf/robot.urdf](config/urdf/robot.urdf)。当前数据
`robot_data/all-data-8-23-4` 对应的平面拟合结果已经写入模型：

```text
base_link -> gps:
  xyz = [-1.193227, -0.405604, 0]
  yaw = +pi/2

livox_imu -> livox_frame:
  xyz = [-0.011000, -0.023290, 0.044120]
  rpy = [0, 0, 0]
```

原始轨迹无法可靠观测 GPS 高度，所以 GPS 的 z 暂为 0；应优先用结构尺寸或专门
外参标定替换它。平面轨迹拟合值也应视为当前数据的估计初值，而不是机械测量真值。

适配另一台车时只需：

1. 按 REP-103（x 前、y 左、z 上）定义 `base_footprint`；
2. 测量或标定各传感器相对 `base_link`/`base_footprint` 的 `xyz/rpy`；
3. 修改 URDF 中对应 fixed joint；
4. 不要在 `mid360.yaml`、GPS YAML 或 EKF YAML 再填写同一外参。

Livox 驱动 JSON 中用于点云预补偿的 `extrinsic_parameter` 必须保持全零；本项目
不会在驱动层提前移动点云，真实 LiDAR–IMU 关系由 URDF 和 FAST-LIO 统一处理。

也可为不同车辆提供另一份 URDF，并在启动时传入
`urdf_file:=/absolute/path/robot.urdf`。

## 构建与启动

```bash
colcon build --symlink-install --packages-select \
  robot_description livox_ros_driver2 fast_lio \
  robot_odom_transform robot_ekf_localization
source install/setup.bash
```

推荐使用隔离回放入口。它在 ROS domain 42 内同时启动定位系统和标准 rosbag2
播放器，避免 Gazebo、实时驱动或可视化回放工具重复发布 `/clock`、LiDAR 和 IMU：

```bash
./localization_replay.sh
```

自定义数据、回放倍率或隔离域：

```bash
./localization_replay.sh \
  bag_path:=/absolute/path/to/bag playback_rate:=1.0 ros_domain_id:=42
```

需要从另一个终端检查话题时，该终端也要先执行 `export ROS_DOMAIN_ID=42`。
不要同时使用 `robot_bag_play_tool`、另一个 `ros2 bag play --clock` 或 Gazebo
时钟参与同一回放域。

实时传感器：

```bash
ros2 launch robot_ekf_localization localization_bringup.launch.py \
  use_sim_time:=false
```

FAST-LIO、GPS 转换和 EKF 的组件 launch 仍可独立运行；独立运行时默认各自加载
URDF。多个组件手动组合时只允许一个发布机器人模型，其余使用
`publish_robot_description:=false`，或者直接使用上面的统一入口。

## 关键话题

| 话题 | 位姿语义 | 用途 |
| --- | --- | --- |
| `/Odometry` | `odom -> livox_imu` | FAST-LIO 原生状态 |
| `/odometry/lio/base` | `odom -> base_footprint` | 杆臂修正后的激光里程计 |
| `/utm/gps` | `utm -> gps` | GPS 天线原生位姿 |
| `/odometry/gps` | `map -> base_footprint` | 杆臂修正后的绝对观测 |
| `/odometry/local` | `odom -> base_footprint` | 局部 EKF |
| `/odometry/global` | `odom -> base_footprint` | 最终融合输出 |

## 验证

```bash
check_urdf config/urdf/robot.urdf
ros2 run tf2_ros tf2_echo base_footprint gps
ros2 run tf2_ros tf2_echo livox_imu livox_frame

ros2 topic echo /Odometry --once
ros2 topic echo /odometry/lio/base --once
ros2 topic echo /odometry/gps --once
ros2 topic hz /odometry/global
```

验证重点不是原始 GPS 天线轨迹与 IMU 轨迹重合，而是转换后的
`/odometry/gps` 与 `/odometry/lio/base` 都描述同一个 `base_footprint`。

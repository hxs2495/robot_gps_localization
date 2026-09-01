# Robot GPS Localization

基于FAST-LIO、GPS和IMU的机器人多传感器融合定位系统

## 项目简介

本项目实现了一个完整的机器人多传感器融合定位方案，通过融合激光雷达里程计（FAST-LIO）、GPS和IMU数据，提供高精度、全局一致的定位能力。

### 核心功能

- **激光SLAM建图与定位**：基于FAST-LIO实现高精度激光里程计
- **GPS数据转换**：将WGS84经纬度坐标转换为本地笛卡尔坐标系
- **双EKF融合架构**：局部EKF平滑里程计，全局EKF融合GPS消除漂移
- **完整的坐标变换链**：维护map → odom → base_footprint → base_link坐标系关系

### 技术特点

- ✅ 短期高精度定位（激光里程计）
- ✅ 长期全局一致性（GPS修正）
- ✅ 平滑连续的位姿估计（EKF滤波）
- ✅ 模块化设计，支持独立测试
- ✅ 完善的文档和配置说明

## 系统架构

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│   Livox激光  │     │   GPS接收机  │     │   IMU传感器  │
│   雷达数据    │     │   /fix话题    │     │  /livox/imu  │
└──────┬───────┘     └──────┬───────┘     └──────┬───────┘
       │                    │                    │
       ▼                    │                    ▼
┌──────────────────┐        │            ┌──────────────┐
│   FAST_LIO       │        │            │  (内置融合)  │
│  激光里程计+IMU  │        │            └──────────────┘
│   /Odometry      │        ▼
└──────┬───────────┘  ┌──────────────────┐
       │              │ robot_odom_      │
       ▼              │ transform        │
┌──────────────────┐  │ /odometry/gps    │
│    局部EKF       │  └────────┬─────────┘
│  /odometry/local │           │
└────────┬─────────┘           │
         └──────────┬──────────┘
                    ▼
            ┌──────────────────┐
            │    全局EKF       │
            │ /odometry/global │
            │  发布map→odom TF │
            └──────────────────┘
```

## 目录结构

```
robot_gps_localization/
├── src/
│   ├── FAST_LIO_ROS2/              # FAST-LIO激光SLAM
│   │   ├── config/                  # 传感器配置
│   │   └── launch/                  # 建图和回环检测启动文件
│   ├── livox_ros_driver2/          # Livox雷达驱动
│   ├── robot_ekf_localization/     # 融合定位核心包
│   │   ├── config/
│   │   │   ├── ekf_local.yaml       # 局部EKF配置
│   │   │   └── ekf_global.yaml      # 全局EKF配置
│   │   ├── launch/
│   │   │   ├── gps_localization.launch.py          # 完整融合系统
│   │   │   ├── local_ekf_localization.launch.py    # 独立局部EKF
│   │   │   └── global_ekf_localization.launch.py   # 独立全局EKF
│   ├── robot_odom_transform/        # GPS坐标与里程计转换
│   └── robot_navigation/           # 机器人导航和URDF模型
└── design/                         # 技术文档
    ├── global_fusion_localization_fix.md           # 全局融合修正文档
    └── 2026-06-18-1430-gps-transform-yaw-calibration.md  # yaw校准文档
```

## 快速开始

### 1. 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+

### 2. 依赖安装

```bash
# 安装robot_localization
sudo apt install ros-humble-robot-localization

# 安装其他依赖
sudo apt install ros-humble-tf2-ros ros-humble-sensor-msgs ros-humble-nav-msgs
```

### 3. 编译

```bash
cd /home/hxs/project/robot_gps_localization
colcon build --symlink-install
source install/setup.bash
```



# FAST_LIO + GPS 全局融合定位

本包使用 `robot_localization` 的双 EKF，将 FAST_LIO 局部里程计与
`robot_odom_transform` 输出的 GPS 里程计融合。

## 数据与 TF 链路

```text
/Odometry (FAST_LIO, odom)
  -> ekf_filter_node_local
  -> /odometry/local + odom -> base_footprint TF
                         \
                          -> ekf_filter_node_global
                         /
/odometry/gps (map)
  -> gps_recovery_smoother
  -> /odometry/gps/smoothed (map) ----+
  -> /odometry/global (map)

ekf_filter_node_global -> 动态 map -> odom TF

/gps_path (map)  <---- GPS 转换轨迹
/fastlio_path (odom) <- FAST_LIO 局部轨迹
```

- 局部 EKF 只使用 FAST_LIO 的位置和姿态。FAST_LIO 本身已经融合 IMU，
  因此不再把同一 IMU 重复送入 EKF。
- 全局 EKF 将局部轨迹作为差分运动约束，保留短时连续性。
- GPS 提供绝对 `x/y/yaw`，用于校正局部轨迹的长期漂移。
- FAST_LIO 当前已关闭自身 TF 广播；局部 EKF 唯一发布连续的
  `odom -> base_footprint`。
- 全局 EKF 在 `world_frame=map` 下动态发布 GPS 修正后的 `map -> odom`；
  请勿启动静态或其他同名 TF 发布者。
- GPS 超时后停止绝对观测，由局部差分继续外推；GPS 恢复后平滑重新锚定。

## 启动

先启动 FAST_LIO 和 GPS 转换：

```bash
ros2 launch robot_odom_transform start_gps_transform.launch.py
```

再启动融合：

```bash
ros2 launch robot_ekf_localization gps_localization.launch.py
```

### rosbag 回放

回放时必须由 rosbag 发布 `/clock`，并让 FAST_LIO、GPS 转换和双 EKF
全部使用仿真时间：

```bash
ros2 bag play <bag_path> --clock
ros2 launch fast_lio mapping.launch.py use_sim_time:=true
ros2 launch robot_odom_transform start_gps_transform.launch.py use_sim_time:=true
ros2 launch robot_ekf_localization gps_localization.launch.py use_sim_time:=true
```

不要在使用 `--clock` 时把传感器时间戳改成系统时间。此时 rosbag 中的
历史时间就是整个定位系统的 ROS 当前时间，`map -> odom` 和
`odom -> base_footprint` 会处于同一时间轴。

可使用以下命令检查：

```bash
ros2 topic echo /clock --once
ros2 param get /laser_mapping use_sim_time
ros2 param get /ekf_filter_node_local use_sim_time
ros2 param get /ekf_filter_node_global use_sim_time
```

用于观察的 RViz 和 rqt 同样应设置：

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
ros2 run rqt_gui rqt_gui --ros-args -p use_sim_time:=true
```

### 实时传感器

实时运行没有 `/clock`，所有节点统一指定：

```bash
ros2 launch fast_lio mapping.launch.py use_sim_time:=false
ros2 launch robot_odom_transform start_gps_transform.launch.py use_sim_time:=false
ros2 launch robot_ekf_localization gps_localization.launch.py use_sim_time:=false
```

## 输入与输出

| 话题               | 坐标系                   | 用途                  |
| ------------------ | ------------------------ | --------------------- |
| `/Odometry`        | `odom -> base_footprint` | FAST_LIO 原始局部位姿 |
| `/odometry/gps`    | `map -> base_footprint`  | GPS 转换后的绝对位姿  |
| `/odometry/gps/smoothed` | `map -> base_footprint` | GPS失效/恢复平滑观测 |
| `/gps_path`        | `map`                     | 转换后的 GPS 路径     |
| `/fastlio_path`    | `odom`                    | FAST_LIO 局部路径     |
| `/odometry/local`  | `odom -> base_footprint` | 局部 EKF 输出         |
| `/odometry/global` | `map -> base_footprint`  | 最终全局融合里程计    |

所有话题均可通过 launch 参数覆盖：

```bash
ros2 launch robot_ekf_localization gps_localization.launch.py \
  lio_odom_topic:=/Odometry \
  gps_odom_topic:=/odometry/gps \
  global_odom_topic:=/odometry/global
```

GPS 路径话题可在转换启动时覆盖：

```bash
ros2 launch robot_odom_transform start_gps_transform.launch.py \
  output_path_topic:=/gps_path
```

路径默认只加入移动距离不小于 `0.05 m` 的点，并最多保留最新 `10000`
个点。可在 GPS 转换参数文件中通过 `path_min_distance` 和
`path_max_points` 调整；将其设为 `0` 分别表示保留每帧和不限制点数。

## 验证

```bash
ros2 topic hz /Odometry
ros2 topic hz /odometry/gps
ros2 topic hz /odometry/gps/smoothed
ros2 topic hz /gps_path
ros2 topic hz /odometry/local
ros2 topic hz /odometry/global

ros2 topic echo /odometry/global --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

全局 EKF 运行后，在 RViz 中将 Fixed Frame 设为 `map`，添加两个 `Path`
显示即可直接对比（局部路径通过动态 `map -> odom` 转换到全局坐标系）：

- `/gps_path`：转换后的 GPS 轨迹；
- `/fastlio_path`：FAST_LIO 局部轨迹。

观察融合结果时可将 Fixed Frame 设为 `map`，同时显示：

- `/odometry/local`：局部轨迹；
- `/odometry/gps`：GPS 观测；
- `/odometry/gps/smoothed`：GPS 消失/恢复状态机输出；
- `/odometry/global`：最终融合轨迹。

## 调参

- GPS 抖动明显：增大 GPS 消息的 `pose.covariance`，或降低 GPS 航向权重。
  默认航向方差在 `robot_odom_transform/config/gps_transform.defaults.yaml`
  的 `orientation_variance` 中配置。
- GPS 跳点：减小 `ekf_global.yaml` 中
  `odom1_pose_rejection_threshold`。
- 正常 GPS 被拒绝：适当增大该阈值，并检查 GPS 协方差是否符合实际精度。
- 不需要 TF、只消费融合里程计时，可以启动参数
  `publish_local_tf:=false publish_global_tf:=false`。

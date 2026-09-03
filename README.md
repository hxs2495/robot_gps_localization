# 融合定位

基于 FAST-LIO、GNSS 航向和 `robot_localization` 的 ROS 2 Humble 融合定位工程。
所有传感器观测先通过 URDF/TF 转换到 `base_footprint`，再进入局部和全局
EKF。GPS 正常时强约束全局位置；GPS 消失时平滑跟随局部定位，恢复时再平滑
贴回 GPS。

## 快速开始

```bash
colcon build --symlink-install --packages-select \
  robot_description livox_ros_driver2 fast_lio robot_odom_transform \
  robot_fusion_path robot_ekf_localization
source install/setup.bash
```

实时运行：

```bash
./localization_bringup.sh use_sim_time:=false
```

统一入口默认发布 `/gps_path`、`/local_path` 和 `/global_path`。

## 融合流程

```text
/livox/lidar + /livox/imu                    /fix + /imu_orientation
             |                                          |
         FAST-LIO                              GPS过滤、同步、UTM转换
             | /Odometry                                | /utm/gps
             v                                          v
      LiDAR参考点TF转换                         GPS杆臂与原点转换
             | /odometry/lio/base                        | /odometry/gps
             v                                          v
          局部EKF                              GPS消失/恢复平滑
             | /odometry/local                           |
             +-------------------+-----------------------+
                                 v
                              全局EKF
                                 |
                         /odometry/global
```

- FAST-LIO 输出 `odom -> livox_imu`，发布层 z 固定为 0。
- LiDAR 和 GPS 均根据 URDF 换算到同一 `base_footprint`。
- 局部 EKF 提供连续位姿增量，GPS 提供绝对位置修正。
- 当前 `map -> odom` 为单位静态 TF，不动态维护。

## 配置入口

| 用途 | 文件 | 主要内容 |
| --- | --- | --- |
| 传感器安装外参 | [`robot.urdf`](config/urdf/robot.urdf) | GPS、LiDAR、IMU 相对车体的 `xyz/rpy` |
| FAST-LIO | [`mid360.yaml`](src/FAST_LIO_ROS2/config/mid360.yaml) | LiDAR/IMU 话题、雷达类型、滤波与发布参数 |
| GPS 转换 | [`gps_transform.defaults.yaml`](src/robot_odom_transform/config/gps_transform.defaults.yaml) | 航向约定、零偏、时延和协方差 |
| 局部 EKF | [`ekf_local.yaml`](src/robot_ekf_localization/config/ekf_local.yaml) | FAST-LIO 平面位姿融合 |
| 全局 EKF | [`ekf_global.yaml`](src/robot_ekf_localization/config/ekf_global.yaml) | GPS 权重、丢失/恢复和过程噪声 |
| 话题与总入口 | [`localization_bringup.launch.py`](src/robot_ekf_localization/launch/localization_bringup.launch.py) | 输入/输出话题、frame 和配置路径 |

**配置原则：**安装位置只改 URDF，话题名只改 launch 参数，滤波效果才改 YAML。

## 话题与坐标系

| 话题 | 语义 | 用途 |
| --- | --- | --- |
| `/livox/lidar` | Livox `CustomMsg` | FAST-LIO 点云输入 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | FAST-LIO IMU 输入 |
| `/fix` | `sensor_msgs/msg/NavSatFix` | GNSS 位置输入 |
| `/imu_orientation` | `sensor_msgs/msg/Imu` | GNSS/双天线航向输入 |
| `/Odometry` | `odom -> livox_imu` | FAST-LIO 原生里程计 |
| `/odometry/lio/base` | `odom -> base_footprint` | 参考点转换后的 LiDAR 里程计 |
| `/odometry/local` | `odom -> base_footprint` | 局部融合定位 |
| `/utm/gps` | `utm -> gps` | GPS 天线 UTM 位姿 |
| `/odometry/gps` | `map -> base_footprint` | 杆臂补偿后的 GPS 观测 |
| `/odometry/gps/smoothed` | `map -> base_footprint` | GPS 消失/恢复状态机输出 |
| `/odometry/global` | `odom -> base_footprint` | 最终全局融合定位 |

送入 EKF 的里程计必须都描述 `base_footprint`，不能直接混合 GPS 天线和
LiDAR/IMU 原点轨迹。

TF 所有权：

```text
robot_state_publisher: base_footprint -> base_link -> {gps, livox_imu}
                                                  livox_imu -> livox_frame
static publisher:      map -> odom
global EKF:            odom -> base_footprint
```

完整系统中只能有一个 `robot_state_publisher` 和一个动态
`odom -> base_footprint` 发布者。

## 调整传感器安装位置

唯一机械外参文件为 [`config/urdf/robot.urdf`](config/urdf/robot.urdf)。
URDF `<origin xyz="..." rpy="..."/>` 表示子 frame 在父 frame 中的位姿
`T_parent_child`，遵循 REP-103：x 前、y 左、z 上，角度单位为弧度。

当前平面外参：

```text
base_link -> livox_imu:   xyz=[0, 0, 0], rpy=[0, 0, 0]
livox_imu -> livox_frame: xyz=[-0.011, -0.02329, 0.04412]
base_link -> gps:         xyz=[-1.193227, -0.405604, 0], yaw=+pi/2
```

更换安装位置：

1. 以车体旋转中心或后轴中心定义 `base_footprint`。
2. 测量传感器相对父 frame 的 x/y/z 和 roll/pitch/yaw。
3. 修改对应 fixed joint，执行 `check_urdf config/urdf/robot.urdf`。
4. 重新构建 `robot_description`，或启动时传入新 URDF。
5. 用直线和转弯数据检查 `/odometry/gps` 与 `/odometry/lio/base`。

```bash
./localization_bringup.sh use_sim_time:=false \
  urdf_file:=/absolute/path/to/vehicle.urdf
```

不要在 Livox 驱动、FAST-LIO YAML 和 URDF 中重复应用同一外参。当前 FAST-LIO
从 TF 读取 `livox_imu -> livox_frame`，`extrinsic_est_en` 保持 `false`。

## GPS 航向与时序

GPS 数据约定在 `gps_transform.defaults.yaml` 中配置：

```yaml
orientation_convention: north_clockwise # 北向0，顺时针为正
orientation_yaw_offset: 0.0297          # 航向固定零偏(rad)
measurement_time_offset: 0.08           # GPS时标补偿(s)
```

`orientation_convention` 也可设为 `enu`（东向0、逆时针为正）。航向零偏和时延不属于
URDF 机械外参；更换 GNSS、航向源或时钟后应重新标定。

## 启动与自定义

更换外部话题：

```bash
./localization_bringup.sh use_sim_time:=false \
  gps_topic:=/gnss/fix \
  orientation_topic:=/gnss/heading \
  lio_sensor_odom_topic:=/fast_lio/odometry
```

加载车辆专用配置：

```bash
./localization_bringup.sh use_sim_time:=false \
  urdf_file:=/absolute/path/to/vehicle.urdf \
  fast_lio_config_path:=/absolute/path/to/config \
  fast_lio_config_file:=mid360.yaml \
  gps_config_file:=/absolute/path/to/gps.yaml \
  local_config_file:=/absolute/path/to/ekf_local.yaml \
  global_config_file:=/absolute/path/to/ekf_global.yaml
```

查看全部参数：

```bash
ros2 launch robot_ekf_localization localization_bringup.launch.py --show-args
```

组件调试入口：

| 脚本 | 功能 |
| --- | --- |
| `fast_lio_mapping.sh` | FAST-LIO |
| `gps_transform.sh` | GPS 过滤、UTM 和杆臂转换 |
| `local_ekf_localization.sh` | LiDAR 参考点转换和局部 EKF |
| `global_ekf_localization.sh` | GPS 状态机和全局 EKF |
| `fusion_path.sh` | local/global Odometry 转 Path |
| `bag_play.sh` | 仅播放 rosbag |

单独调试组件时可使用这些脚本；完整运行时不要与统一入口重复启动。

## 验收

```bash
check_urdf config/urdf/robot.urdf
ros2 run tf2_ros tf2_echo base_footprint gps
ros2 run tf2_ros tf2_echo livox_imu livox_frame

ros2 topic echo /odometry/lio/base --once
ros2 topic echo /odometry/gps --once
ros2 topic hz /odometry/local
ros2 topic hz /odometry/global
```

预期：

- `/odometry/lio/base` 和 `/odometry/gps` 的 child 均为 `base_footprint`。
- z、roll、pitch 在融合发布层为 0。
- GPS 正常时 global 贴合 GPS，GPS 消失时 global 连续跟随 local。
- RViz2 中 `/gps_path`、`/local_path`、`/global_path` 方向一致。

常见现象：

| 现象 | 检查项 |
| --- | --- |
| 越远离起点横向误差越大 | `orientation_yaw_offset` |
| 仅转弯时两条轨迹错开 | URDF 杆臂、`measurement_time_offset` |
| GPS 恢复时跳变 | `ekf_global.yaml` 恢复时长和校正限速 |
| TF 抖动或 multiple authority | 重复启动的节点或 TF 发布者 |
| 回放时节点超时 | 多个 `/clock`、`use_sim_time` 错误或回放倍速过高 |

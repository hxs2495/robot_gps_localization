# FAST_LIO + GPS 全局融合定位

本包使用 `robot_localization` 的双 EKF，将 FAST_LIO 局部里程计与
`robot_odom_transform` 输出的 GPS 里程计融合。

## 数据与 TF 链路

```text
/Odometry (FAST_LIO, odom)
  -> ekf_filter_node_local
  -> /odometry/local
  -> odom -> base_footprint TF
                         \
                          -> ekf_filter_node_global
                         /
/odometry/gps (map) ----+
  -> /odometry/global (map)
  -> map -> odom TF
```

- 局部 EKF 只使用 FAST_LIO 的位置和姿态。FAST_LIO 本身已经融合 IMU，
  因此不再把同一 IMU 重复送入 EKF。
- 全局 EKF 将局部轨迹作为差分运动约束，保留短时连续性。
- GPS 提供绝对 `x/y/yaw`，用于校正局部轨迹的长期漂移。
- FAST_LIO 当前已关闭自身 TF 广播；`odom -> base_footprint` 由局部 EKF
  唯一发布，`map -> odom` 由全局 EKF 唯一发布。
- `map -> odom` 是全局观测对局部漂移的动态校正，不应再发布同名静态变换。
  启动时两者应近似重合，之后该变换可以缓慢变化，但不应高频来回跳动。

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

回放时必须由 rosbag 发布 `/clock`，并确保所有处理节点都启用
`use_sim_time=true`。项目中的 FAST_LIO、GPS 转换和双 EKF launch
现在均默认使用该模式：

```bash
ros2 bag play <bag_path> --clock
ros2 launch fast_lio mapping.launch.py use_sim_time:=true
ros2 launch robot_odom_transform start_gps_transform.launch.py use_sim_time:=true
ros2 launch robot_ekf_localization gps_localization.launch.py use_sim_time:=true
```

如果局部 EKF 已单独启动，则最后一条可替换为：

```bash
ros2 launch robot_ekf_localization local_ekf_localization.launch.py \
  use_sim_time:=true
ros2 launch robot_ekf_localization global_ekf_localization.launch.py \
  use_sim_time:=true
```

不要把 FAST_LIO 输出时间戳改成系统时间。传感器消息保留录制时间，
`/clock` 让各节点的 ROS 当前时间进入同一条回放时间轴。

检查所有节点是否使用同一时钟：

```bash
ros2 topic echo /clock --once
ros2 param get /laser_mapping use_sim_time
ros2 param get /gps_global_odometry_node use_sim_time
ros2 param get /ekf_filter_node_local use_sim_time
ros2 param get /ekf_filter_node_global use_sim_time
```

以上参数都应返回 `True`。

用于观察 TF 时，RViz 和 rqt 也应使用回放时钟：

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
ros2 run rqt_gui rqt_gui --ros-args -p use_sim_time:=true
```

### 实时传感器

连接实时雷达和 GPS 时没有 rosbag `/clock`，所有节点统一改为系统时间：

```bash
ros2 launch fast_lio mapping.launch.py use_sim_time:=false
ros2 launch robot_odom_transform start_gps_transform.launch.py \
  use_sim_time:=false
ros2 launch robot_ekf_localization gps_localization.launch.py \
  use_sim_time:=false
```

## 输入与输出

| 话题 | 坐标系 | 用途 |
|---|---|---|
| `/Odometry` | `odom -> base_footprint` | FAST_LIO 原始局部位姿 |
| `/odometry/gps` | `map -> base_footprint` | GPS 转换后的绝对位姿 |
| `/odometry/local` | `odom -> base_footprint` | 局部 EKF 输出 |
| `/odometry/global` | `map -> base_footprint` | 最终全局融合里程计 |

所有话题均可通过 launch 参数覆盖：

```bash
ros2 launch robot_ekf_localization gps_localization.launch.py \
  lio_odom_topic:=/Odometry \
  gps_odom_topic:=/odometry/gps \
  global_odom_topic:=/odometry/global
```

如果局部 EKF 已经单独运行，也可以只启动全局 EKF：

```bash
ros2 launch robot_ekf_localization global_ekf_localization.launch.py
```

启动后应确认 GPS 消息属于 `map`：

```bash
ros2 topic echo /odometry/gps --once --field header.frame_id
```

## 验证

```bash
ros2 topic hz /Odometry
ros2 topic hz /odometry/gps
ros2 topic hz /odometry/local
ros2 topic hz /odometry/global

ros2 topic echo /odometry/global --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

RViz 中将 Fixed Frame 设为 `map`，同时显示：

- `/odometry/local`：局部轨迹；
- `/odometry/gps`：GPS 观测；
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

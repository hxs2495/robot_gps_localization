# robot_ekf_localization

该包只保留基于 `robot_localization` 的局部、全局双 EKF 融合链路。
GPS 经纬度转换由 `robot_odom_transform` 负责，本包只消费转换后的 GPS 里程计。

## 数据与 TF 链路

```text
/Odometry (FAST-LIO, odom)
  -> ekf_filter_node_local
  -> /odometry/local (odom)
  -> odom -> base_footprint TF
                         \
                          -> ekf_filter_node_global
                         /
/odometry/gps (map) ----+
  -> /odometry/global (map)
  -> map -> odom TF
```

- 局部 EKF 只融合 FAST-LIO 的平面 `x/y/yaw`，输出连续的局部里程计；
- 全局 EKF 将局部轨迹作为差分运动约束，并用 GPS 的绝对 `x/y/yaw` 校正漂移；
- FAST-LIO 已经融合 IMU，本包不重复融合同一 IMU；
- 局部 EKF 唯一发布 `odom -> base_footprint`，全局 EKF 唯一发布动态
  `map -> odom`，不能再发布同名静态 TF。

旧版 `navsat_transform`、Cartographer EKF、GPS 时间戳改写及失效测试脚本已经移除。

## 启动

完整双融合：

```bash
ros2 launch robot_ekf_localization gps_localization.launch.py
```

也可以独立启动其中一级：

```bash
ros2 launch robot_ekf_localization local_ekf_localization.launch.py
ros2 launch robot_ekf_localization global_ekf_localization.launch.py
```

双融合启动前需要外部提供：

- `/Odometry`：FAST-LIO 局部里程计，位于 `odom`；
- `/odometry/gps`：`robot_odom_transform` 输出的 GPS 绝对里程计，位于 `map`。

常用接口均可覆盖：

```bash
ros2 launch robot_ekf_localization gps_localization.launch.py \
  lio_odom_topic:=/Odometry \
  gps_odom_topic:=/odometry/gps \
  local_odom_topic:=/odometry/local \
  global_odom_topic:=/odometry/global
```

不需要 TF 时可附加：

```text
publish_local_tf:=false publish_global_tf:=false
```

## 时间模式

launch 默认 `use_sim_time:=true`，适用于项目当前的 rosbag 回放流程：

```bash
ros2 bag play <bag_path> --clock
ros2 launch fast_lio mapping.launch.py use_sim_time:=true
ros2 launch robot_odom_transform start_gps_transform.launch.py use_sim_time:=true
ros2 launch robot_ekf_localization gps_localization.launch.py use_sim_time:=true
```

实时传感器运行时，所有处理节点必须统一使用系统时间：

```bash
ros2 launch robot_ekf_localization gps_localization.launch.py use_sim_time:=false
```

不要用当前系统时间覆盖 rosbag 消息的原始时间戳，否则 TF 和传感器数据会落在
不同时间轴上。

## 配置

- `config/ekf_local.yaml`：局部 EKF，`world_frame=odom`；
- `config/ekf_global.yaml`：全局 EKF，`world_frame=map`。

全局 EKF 的 GPS 输入必须满足：

```text
/odometry/gps.header.frame_id == map
```

否则可能形成 `map -> odom` 观测反馈环。

## 验证与调参

```bash
ros2 topic hz /Odometry
ros2 topic hz /odometry/gps
ros2 topic hz /odometry/local
ros2 topic hz /odometry/global
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

- GPS 抖动明显：增大 GPS 消息的 `pose.covariance`；
- GPS 跳点未被拒绝：减小 `ekf_global.yaml` 的
  `odom1_pose_rejection_threshold`；
- 正常 GPS 被拒绝：适当增大该阈值，并检查 GPS 协方差是否符合实际精度。

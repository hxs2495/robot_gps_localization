# robot_odom_transform

本包将 `sensor_msgs/NavSatFix` 转换为机器人全局坐标系中的
`nav_msgs/Odometry`，用于 RViz 轨迹显示，也可作为后续全局 EKF 的 GPS
位置观测。

## 数据链路

```text
/fix
  -> gpgga_filter_node             -> /fix/filter
  -> gps_to_utm_odometry_node      -> /utm/gps       (T_utm_gps)
  -> gps_global_odometry_node      -> /odometry/gps  (T_odom_base_footprint)
```

`gps_global_odometry_node` 会：

1. 使用标定外参 `T_gps_base`，将 GPS 天线位姿换算成机器人基座位姿；
2. 对启动时的 `init_frames` 帧求平均，以机器人的初始位置和航向建立局部
   全局原点；
3. 将后续 GPS 位姿、姿态和位姿协方差旋转到 `global_frame_id`；
4. 默认输出 `/odometry/gps`，但不发布 TF，避免与 FAST-LIO 或
   `robot_localization` 重复发布 `odom -> base_footprint`。

默认 `global_frame_id=odom`，与当前 FAST-LIO 的全局坐标系一致。若融合系统
要求 GPS 观测位于 `map`，启动时传入 `global_frame_id:=map`，并确保局部
里程计与该初始化约定一致。

## 启动

```bash
source install/setup.bash
ros2 launch robot_odom_transform start_gps_transform.launch.py
```

需要外部提供：

- `/fix`：GPS 定位；
- `/imu_orientation`：双天线 GNSS 或已转换到 ENU 约定的航向。默认会等待
  首帧航向，避免用未初始化角度建立全局坐标系。

在 RViz 中将 Fixed Frame 设为 `odom`，添加 Odometry 显示并选择
`/odometry/gps`。若 FAST-LIO 的 `/Odometry` 也位于 `odom`，两条轨迹可直接
对比。

常用话题和坐标系均可从 launch 覆盖：

```bash
ros2 launch robot_odom_transform start_gps_transform.launch.py \
  gps_topic:=/gps/fix \
  orientation_topic:=/gnss/heading \
  output_odom_topic:=/odometry/gps \
  global_frame_id:=odom \
  child_frame_id:=base_footprint
```

## 标定配置与默认回退

包内默认配置为 `config/gps_transform.defaults.yaml`，其中包含默认小车
`T_gps_base` 标定。`config_file` 未传、为空，或指定文件不存在时，launch
都会使用该默认配置；节点不再依赖写死的用户目录。

自定义时可复制默认文件，只修改对应参数：

```yaml
gps_global_odometry_node:
  ros__parameters:
    init_frames: 10
    two_d_mode: true
    publish_tf: false
    gps_to_base:
      x: 0.726732
      y: -0.109451
      z: 0.0
      qx: 0.00259991
      qy: 0.00764529
      qz: -0.257884
      qw: 0.966142
```

启动自定义标定：

```bash
ros2 launch robot_odom_transform start_gps_transform.launch.py \
  config_file:=/absolute/path/to/gps_transform.yaml
```

外参约定必须是 `T_gps_base`：父坐标系为 GPS 天线，子坐标系为机器人
`base_footprint`。四元数顺序为 `x, y, z, w`。

若没有航向话题，可以在自定义 YAML 中设置：

```yaml
gps_to_utm_odometry_node:
  ros__parameters:
    use_orientation: false
    require_orientation: false
```

此时轨迹仍能显示，但全局坐标轴仅按 UTM 东北方向建立，可能无法与启动方向
任意的局部激光里程计轨迹对齐。

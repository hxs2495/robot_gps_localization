# robot_odom_transform

该包只保留 GPS 定位数据的核心转换链路：过滤 `NavSatFix`，转换到 UTM，
再结合 GPS 天线到机器人基座的外参生成局部化的全局里程计。

## 核心链路

```text
/fix                         sensor_msgs/NavSatFix
  -> gpgga_filter_node
/fix/filter                  sensor_msgs/NavSatFix
  -> gps_to_utm_odometry_node  (+ /imu_orientation)
/utm/gps                     nav_msgs/Odometry, T_utm_gps
  -> gps_global_odometry_node  (+ T_gps_base)
/odometry/gps                nav_msgs/Odometry, T_map_base_footprint
```

三个节点的职责如下：

- `gpgga_filter_node`：检查坐标、定位状态和协方差，并在连续有效帧达到要求后发布；
- `gps_to_utm_odometry_node`：执行 WGS84 到 UTM 转换，注入可选航向并生成合理的位姿协方差；
- `gps_global_odometry_node`：应用 `T_gps_base` 外参，以启动阶段的平均位姿建立局部原点，
  将结果和协方差旋转到 `global_frame_id`。

默认不发布 TF，避免与 FAST-LIO 或 `robot_localization` 重复发布全局坐标系到
`base_footprint` 的变换。`/gps_path` 仍作为现有 RViz 配置所需的轻量可视化输出。

旧版反向 GPS 转换、重复滤波器、TF 拼接、外参落盘、零点化和测试节点已移除；
现行链路不再依赖 `robot_interfaces` 的 UTM 分区消息。

## 启动

```bash
source install/setup.bash
ros2 launch robot_odom_transform start_gps_transform.launch.py
```

外部输入：

- `/fix`：GPS 定位；
- `/imu_orientation`：双天线 GNSS 或已按当前项目约定转换的航向。

常用接口可从 launch 覆盖：

```bash
ros2 launch robot_odom_transform start_gps_transform.launch.py \
  gps_topic:=/gps/fix \
  orientation_topic:=/gnss/heading \
  output_odom_topic:=/odometry/gps \
  global_frame_id:=map \
  child_frame_id:=base_footprint
```

回放 rosbag 时附加 `use_sim_time:=true`。

## 配置

默认配置位于 `config/gps_transform.defaults.yaml`。`config_file` 为空或文件不存在时，
launch 会回退到这个文件。自定义配置示例：

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
      qx: 0.0
      qy: 0.0
      qz: -0.70710678
      qw: 0.70710678
```

其中外参定义为 `T_gps_base`：父坐标系是 GPS 天线，子坐标系是机器人基座，
四元数顺序为 `x, y, z, w`。

```bash
ros2 launch robot_odom_transform start_gps_transform.launch.py \
  config_file:=/absolute/path/to/gps_transform.yaml
```

没有航向输入时，可设置：

```yaml
gps_to_utm_odometry_node:
  ros__parameters:
    use_orientation: false
    require_orientation: false
```

此时位置转换仍可工作，但输出坐标轴按 UTM 东北方向建立，不能保证与启动方向任意的
局部激光里程计轨迹对齐。

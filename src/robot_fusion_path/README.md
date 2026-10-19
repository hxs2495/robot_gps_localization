# robot_fusion_path

该包提供两类最终输出：

- `fusion_path_node`：把 local/global Odometry 累积为 RViz Path；
- `global_localization_node`：用首个时序对齐的有效 GPS、航向和全局融合
  Odometry 建立全球锚点，之后把每帧 `/odometry/global` 转成 WGS84 全球定位。

## 全球定位接口

默认订阅：

```text
/fix/filter       sensor_msgs/msg/NavSatFix
/imu_orientation sensor_msgs/msg/Imu
/odometry/global nav_msgs/msg/Odometry
TF: gps <- base_footprint
```

默认发布：

```text
/localization/global     robot_fusion_path/msg/GlobalLocalization
/localization/global_fix sensor_msgs/msg/NavSatFix
```

自定义消息包含经度、纬度、椭球高、ENU 姿态、车体坐标系速度、位姿/速度
协方差、锚点 GPS 状态以及预留的卫星数字段。`NavSatFix` 输出是融合结果的
兼容表示，并非一帧新的 GNSS 原始测量。
该兼容消息的 `header.frame_id` 是 `base_footprint`，因为经纬高已经做过
GPS 天线杆臂补偿。

初始化使用：

```text
T_enu_map = T_enu_base(anchor) * inverse(T_map_base(anchor))
T_enu_base(now) = T_enu_map * T_map_base(now)
```

其中 `T_enu_base(anchor)` 会通过 URDF 中的 `gps <- base_footprint` 静态 TF
补偿天线杆臂。GPS、航向和融合里程计的时间差必须小于
`max_anchor_time_offset`，避免车辆运动中用错时数据初始化。

`orientation_convention` 和 `orientation_yaw_offset` 必须与
`gps_to_utm_odometry_node` 的 GPS 转换配置一致；本项目默认分别为
`north_clockwise` 和 `0.0297` rad。

单天线且没有绝对航向时，可以设置 `use_orientation:=false` 并通过
`initial_enu_yaw` 给出启动时车体 x 轴相对正东、逆时针为正的角度；否则仅凭
静止单天线 GPS 无法确定全球姿态。

单独启动：

```bash
ros2 launch robot_fusion_path fusion_path.launch.py \
  use_sim_time:=false publish_paths:=false
```

查看输出：

```bash
ros2 topic echo /localization/global --once
ros2 topic echo /localization/global_fix --once
```

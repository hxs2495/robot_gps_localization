# robot_odom_transform

该包把传感器原点上的里程计转换到统一的机器人融合参考点。安装外参不再作为
节点参数或 YAML 数值保存，全部在运行时从 URDF 发布的静态 TF 查询。

## 数据链路

```text
/fix -> gpgga_filter_node -> /fix/filter
  + 同时刻 /imu_orientation
  -> gps_to_utm_odometry_node -> /utm/gps       (utm -> gps)
  + TF: gps <-> base_footprint
  -> gps_global_odometry_node -> /odometry/gps  (map -> base_footprint)

/Odometry                                            (odom -> livox_imu)
  + TF: livox_imu <-> base_footprint
  -> odometry_tf_transform_node -> /odometry/lio/base (odom -> base_footprint)
```

`gps_to_utm_odometry_node` 使用近似时间同步配对位置和航向，避免拿上一周期航向
处理当前 GPS。`gps_global_odometry_node` 对 GPS 杆臂做完整刚体变换；通用里程计
转换节点还会同步变换 twist、位姿协方差和速度协方差。

数学上，若传感器里程计为 `T_parent_sensor`，目标参考点由 TF 给出
`T_sensor_base`，则输出为：

```text
T_parent_base = T_parent_sensor * T_sensor_base
```

因此转弯时 GPS 天线、IMU 和车体参考点的轨迹可以不同，但转换后的两种观测都
表示同一个 `base_footprint`，才能送进同一个 EKF。

## 启动

GPS 链路独立启动时会自动加载项目 URDF：

```bash
source install/setup.bash
ros2 launch robot_odom_transform start_gps_transform.launch.py \
  use_sim_time:=true
```

实时设备使用 `use_sim_time:=false`。完整系统建议使用统一入口：

```bash
ros2 launch robot_ekf_localization localization_bringup.launch.py
```

多个组件由外部统一启动时，只有一个组件应发布机器人模型，其余附加：

```text
publish_robot_description:=false
```

## 配置边界

`config/gps_transform.defaults.yaml` 只包含数据过滤、时间同步、协方差、初始化和
路径采样等运行参数。以下参数已经删除，不能再写入配置文件：

```text
gps_to_base.x/y/z/qx/qy/qz/qw
```

GPS 安装位置和方向只能修改 `config/urdf/robot.urdf` 中
`base_link_to_gps` 的 `<origin>`。修改后重新构建 `robot_description`，或者用
launch 参数 `urdf_file:=/absolute/path/robot.urdf` 直接加载另一台车的模型。

没有航向输入时可将 `use_orientation` 设为 `false`。此时位置仍可转换，但不能
仅凭单天线静止 GPS 确定机器人朝向。

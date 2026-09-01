#!/usr/bin/env python3
"""Smooth GPS recovery while local odometry propagates through outages."""

import math
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def smooth_step(value):
    """Return a cubic interpolation ratio with zero endpoint slopes."""
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def interpolate_pose(source, target, ratio):
    """Interpolate an SE(2) pose represented as x, y and yaw."""
    ratio = max(0.0, min(1.0, ratio))
    return (
        source[0] + (target[0] - source[0]) * ratio,
        source[1] + (target[1] - source[1]) * ratio,
        normalize_angle(
            source[2] + normalize_angle(target[2] - source[2]) * ratio
        ),
    )


def compose_pose(reference_to_local, local_to_body):
    """Compose T_reference_local with T_local_body."""
    cosine = math.cos(reference_to_local[2])
    sine = math.sin(reference_to_local[2])
    return (
        reference_to_local[0]
        + cosine * local_to_body[0]
        - sine * local_to_body[1],
        reference_to_local[1]
        + sine * local_to_body[0]
        + cosine * local_to_body[1],
        normalize_angle(reference_to_local[2] + local_to_body[2]),
    )


def calculate_correction(global_pose, local_pose):
    """Calculate T_map_odom from simultaneous map and odom poses."""
    yaw = normalize_angle(global_pose[2] - local_pose[2])
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        global_pose[0] - cosine * local_pose[0] + sine * local_pose[1],
        global_pose[1] - sine * local_pose[0] - cosine * local_pose[1],
        yaw,
    )


def extract_pose(message):
    """Extract a finite planar pose from an odometry message."""
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    norm = math.sqrt(
        orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
        + orientation.w * orientation.w
    )
    if norm < 1.0e-12 or not all(
        math.isfinite(value) for value in (position.x, position.y, norm)
    ):
        return None
    x = orientation.x / norm
    y = orientation.y / norm
    z = orientation.z / norm
    w = orientation.w / norm
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    if not math.isfinite(yaw):
        return None
    return (position.x, position.y, yaw)


class GpsRecoverySmoother(Node):
    """Publish GPS observations only while valid, with smooth reacquisition."""

    WAITING = 'waiting'
    RECOVERING = 'recovering'
    TRACKING = 'tracking'
    LOCAL_FALLBACK = 'local_fallback'

    def __init__(self):
        super().__init__('gps_recovery_smoother')
        self.declare_parameter('raw_gps_topic', '/odometry/gps')
        self.declare_parameter('local_odom_topic', '/odometry/local')
        self.declare_parameter('output_topic', '/odometry/gps/smoothed')
        self.declare_parameter('output_frame_id', 'map')
        self.declare_parameter('gps_timeout', 1.0)
        self.declare_parameter('loss_transition_duration', 3.0)
        self.declare_parameter('local_fallback_covariance_scale', 1.0)
        self.declare_parameter('recovery_duration', 5.0)
        self.declare_parameter('correction_time_constant', 0.5)
        self.declare_parameter('recovery_covariance_scale', 10.0)
        self.declare_parameter('tracking_covariance_scale', 0.01)
        self.declare_parameter('tracking_covariance_ramp_duration', 1.0)
        self.declare_parameter('max_translation_correction_rate', 1.0)
        self.declare_parameter('max_yaw_correction_rate', 0.35)
        self.declare_parameter('tracking_position_tolerance', 0.05)
        self.declare_parameter('tracking_yaw_tolerance', 0.02)
        self.declare_parameter('max_local_time_offset', 0.5)

        self.output_frame_id = self.get_parameter(
            'output_frame_id'
        ).value
        self.gps_timeout = max(0.01, self.get_parameter('gps_timeout').value)
        self.loss_transition_duration = max(
            0.0, self.get_parameter('loss_transition_duration').value
        )
        self.local_fallback_covariance_scale = max(
            1.0e-6,
            self.get_parameter('local_fallback_covariance_scale').value,
        )
        self.recovery_duration = max(
            0.0, self.get_parameter('recovery_duration').value
        )
        self.correction_time_constant = max(
            0.0, self.get_parameter('correction_time_constant').value
        )
        self.recovery_covariance_scale = max(
            1.0, self.get_parameter('recovery_covariance_scale').value
        )
        self.tracking_covariance_scale = max(
            1.0e-6, self.get_parameter('tracking_covariance_scale').value
        )
        self.tracking_covariance_ramp_duration = max(
            0.0,
            self.get_parameter('tracking_covariance_ramp_duration').value,
        )
        self.max_translation_correction_rate = max(
            0.0,
            self.get_parameter('max_translation_correction_rate').value,
        )
        self.max_yaw_correction_rate = max(
            0.0, self.get_parameter('max_yaw_correction_rate').value
        )
        self.tracking_position_tolerance = max(
            0.0, self.get_parameter('tracking_position_tolerance').value
        )
        self.tracking_yaw_tolerance = max(
            0.0, self.get_parameter('tracking_yaw_tolerance').value
        )
        self.max_local_time_offset = max(
            0.0, self.get_parameter('max_local_time_offset').value
        )

        output_topic = self.get_parameter('output_topic').value
        local_topic = self.get_parameter('local_odom_topic').value
        gps_topic = self.get_parameter('raw_gps_topic').value
        self.output_publisher = self.create_publisher(Odometry, output_topic, 10)
        self.create_subscription(
            Odometry, local_topic, self.local_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, gps_topic, self.gps_callback, qos_profile_sensor_data
        )
        self.create_timer(0.1, self.check_gps_timeout)

        self.local_history = deque(maxlen=500)
        self.mode = self.WAITING
        self.current_correction = (0.0, 0.0, 0.0)
        self.recovery_start_correction = self.current_correction
        self.filtered_target_correction = self.current_correction
        self.fallback_start_correction = self.current_correction
        self.last_gps_receive_ns = None
        self.recovery_start_ns = 0
        self.tracking_start_ns = 0
        self.fallback_start_ns = 0
        self.last_update_ns = 0
        self.have_ever_tracked = False

        self.get_logger().info(
            f'GPS状态机已启动: {gps_topic} + {local_topic} -> '
            f'{output_topic}; GPS超时后平滑贴回局部定位'
        )

    def now_ns(self):
        """Return current ROS time in nanoseconds."""
        return self.get_clock().now().nanoseconds

    @staticmethod
    def stamp_ns(message):
        """Convert a ROS message timestamp to nanoseconds."""
        return (
            message.header.stamp.sec * 1000000000
            + message.header.stamp.nanosec
        )

    def local_callback(self, message):
        """Cache local poses for GPS timestamp alignment."""
        pose = extract_pose(message)
        if pose is None:
            return
        stamp = self.stamp_ns(message)
        if self.local_history and stamp < self.local_history[-1][0]:
            self.local_history.clear()
            self.get_logger().warning('局部里程计时间回跳，已清空对齐缓存')
        self.local_history.append((stamp, pose))
        while (
            self.local_history
            and stamp - self.local_history[0][0] > 5000000000
        ):
            self.local_history.popleft()
        if self.mode == self.LOCAL_FALLBACK:
            self.publish_local_fallback(message, pose)

    def gps_callback(self, message):
        """Smooth GPS reacquisition and publish a valid absolute observation."""
        gps_pose = extract_pose(message)
        if gps_pose is None or not self.local_history:
            return
        gps_stamp = self.stamp_ns(message)
        local_stamp, local_pose = min(
            self.local_history, key=lambda item: abs(gps_stamp - item[0])
        )
        if abs(gps_stamp - local_stamp) * 1.0e-9 > self.max_local_time_offset:
            return

        now = self.now_ns()
        time_jumped = (
            self.last_gps_receive_ns is not None
            and now < self.last_gps_receive_ns
        )
        timed_out = (
            self.last_gps_receive_ns is not None
            and not time_jumped
            and (now - self.last_gps_receive_ns) * 1.0e-9 > self.gps_timeout
        )
        start_recovery = (
            self.mode in (self.WAITING, self.LOCAL_FALLBACK)
            or timed_out
            or time_jumped
        )
        self.last_gps_receive_ns = now
        target_correction = calculate_correction(gps_pose, local_pose)
        recovery_progress = 1.0

        if start_recovery:
            self.recovery_start_correction = self.current_correction
            self.filtered_target_correction = target_correction
            self.recovery_start_ns = now
            self.last_update_ns = now
            self.mode = self.RECOVERING
            recovery_progress = 0.0
            status = '恢复' if self.have_ever_tracked else '首次有效'
            self.get_logger().info(
                f'GPS{status}，开始在{self.recovery_duration:.2f}s内平滑锚定'
            )
        else:
            update_dt = max(0.0, (now - self.last_update_ns) * 1.0e-9)
            if self.mode == self.RECOVERING:
                ratio = (
                    1.0
                    if self.correction_time_constant <= 0.0
                    else 1.0
                    - math.exp(-update_dt / self.correction_time_constant)
                )
                self.filtered_target_correction = interpolate_pose(
                    self.filtered_target_correction,
                    target_correction,
                    ratio,
                )
                recovery_progress = (
                    1.0
                    if self.recovery_duration <= 0.0
                    else max(
                        0.0,
                        min(
                            1.0,
                            (now - self.recovery_start_ns)
                            * 1.0e-9
                            / self.recovery_duration,
                        ),
                    )
                )
                desired = (
                    target_correction
                    if recovery_progress >= 1.0
                    else interpolate_pose(
                        self.recovery_start_correction,
                        self.filtered_target_correction,
                        smooth_step(recovery_progress),
                    )
                )
                self.current_correction = self.limit_correction(
                    self.current_correction, desired, update_dt
                )
                position_error = math.hypot(
                    target_correction[0] - self.current_correction[0],
                    target_correction[1] - self.current_correction[1],
                )
                yaw_error = abs(
                    normalize_angle(
                        target_correction[2] - self.current_correction[2]
                    )
                )
                if (
                    recovery_progress >= 1.0
                    and position_error <= self.tracking_position_tolerance
                    and yaw_error <= self.tracking_yaw_tolerance
                ):
                    self.current_correction = target_correction
                    self.mode = self.TRACKING
                    self.tracking_start_ns = now
                    self.have_ever_tracked = True
                    self.get_logger().info('GPS恢复完成，进入高置信度跟踪')
            else:
                self.current_correction = target_correction
            self.last_update_ns = now

        self.publish_observation(
            message, gps_pose, local_pose, recovery_progress, now
        )

    def limit_correction(self, current, desired, delta_time):
        """Rate-limit translation and yaw corrections."""
        if delta_time <= 0.0:
            return current
        delta_x = desired[0] - current[0]
        delta_y = desired[1] - current[1]
        distance = math.hypot(delta_x, delta_y)
        maximum_distance = self.max_translation_correction_rate * delta_time
        if (
            self.max_translation_correction_rate > 0.0
            and distance > maximum_distance
            and distance > 1.0e-12
        ):
            ratio = maximum_distance / distance
            output_x = current[0] + delta_x * ratio
            output_y = current[1] + delta_y * ratio
        else:
            output_x = desired[0]
            output_y = desired[1]
        delta_yaw = normalize_angle(desired[2] - current[2])
        maximum_yaw = self.max_yaw_correction_rate * delta_time
        if self.max_yaw_correction_rate > 0.0:
            delta_yaw = max(-maximum_yaw, min(maximum_yaw, delta_yaw))
        return (output_x, output_y, normalize_angle(current[2] + delta_yaw))

    def publish_observation(
        self, raw_gps, gps_pose, local_pose, recovery_progress, now
    ):
        """Publish exact GPS in tracking or a continuous recovery pose."""
        output = Odometry()
        output.header = raw_gps.header
        output.header.frame_id = self.output_frame_id
        output.child_frame_id = raw_gps.child_frame_id
        output.pose = raw_gps.pose
        output.twist = raw_gps.twist

        if self.mode == self.TRACKING:
            output_pose = gps_pose
            tracking_progress = (
                1.0
                if self.tracking_covariance_ramp_duration <= 0.0
                else max(
                    0.0,
                    min(
                        1.0,
                        (now - self.tracking_start_ns)
                        * 1.0e-9
                        / self.tracking_covariance_ramp_duration,
                    ),
                )
            )
            covariance_scale = 1.0 + (
                self.tracking_covariance_scale - 1.0
            ) * smooth_step(tracking_progress)
        else:
            output_pose = compose_pose(self.current_correction, local_pose)
            covariance_scale = 1.0 + (
                self.recovery_covariance_scale - 1.0
            ) * (1.0 - smooth_step(recovery_progress))

        output.pose.pose.position.x = output_pose[0]
        output.pose.pose.position.y = output_pose[1]
        output.pose.pose.position.z = 0.0
        output.pose.pose.orientation.x = 0.0
        output.pose.pose.orientation.y = 0.0
        output.pose.pose.orientation.z = math.sin(output_pose[2] * 0.5)
        output.pose.pose.orientation.w = math.cos(output_pose[2] * 0.5)
        output.pose.covariance = [
            value * covariance_scale if math.isfinite(value) else value
            for value in output.pose.covariance
        ]
        self.output_publisher.publish(output)

    def publish_local_fallback(self, local_odometry, local_pose):
        """Publish a smooth absolute observation that converges to local pose."""
        now = self.now_ns()
        progress = (
            1.0
            if self.loss_transition_duration <= 0.0
            else max(
                0.0,
                min(
                    1.0,
                    (now - self.fallback_start_ns)
                    * 1.0e-9
                    / self.loss_transition_duration,
                ),
            )
        )
        self.current_correction = interpolate_pose(
            self.fallback_start_correction,
            (0.0, 0.0, 0.0),
            smooth_step(progress),
        )
        output_pose = compose_pose(self.current_correction, local_pose)

        output = Odometry()
        output.header = local_odometry.header
        output.header.frame_id = self.output_frame_id
        output.child_frame_id = local_odometry.child_frame_id
        output.pose = local_odometry.pose
        output.twist = local_odometry.twist
        output.pose.pose.position.x = output_pose[0]
        output.pose.pose.position.y = output_pose[1]
        output.pose.pose.position.z = 0.0
        output.pose.pose.orientation.x = 0.0
        output.pose.pose.orientation.y = 0.0
        output.pose.pose.orientation.z = math.sin(output_pose[2] * 0.5)
        output.pose.pose.orientation.w = math.cos(output_pose[2] * 0.5)
        output.pose.covariance = [
            value * self.local_fallback_covariance_scale
            if math.isfinite(value) else value
            for value in output.pose.covariance
        ]
        self.output_publisher.publish(output)

    def check_gps_timeout(self):
        """Detect GPS loss and start a smooth transition to local pose."""
        if self.last_gps_receive_ns is None or self.mode in (
            self.WAITING,
            self.LOCAL_FALLBACK,
        ):
            return
        now = self.now_ns()
        if (
            now < self.last_gps_receive_ns
            or (now - self.last_gps_receive_ns) * 1.0e-9 > self.gps_timeout
        ):
            self.fallback_start_correction = self.current_correction
            self.fallback_start_ns = now
            self.mode = self.LOCAL_FALLBACK
            self.get_logger().warning(
                'GPS超时：开始平滑贴回局部融合定位'
            )


def main(args=None):
    """Run the GPS recovery smoother node."""
    rclpy.init(args=args)
    node = GpsRecoverySmoother()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

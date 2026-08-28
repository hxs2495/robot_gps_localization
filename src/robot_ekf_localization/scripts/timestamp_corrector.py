#!/usr/bin/env python3
"""
GPS时间戳修正节点
将历史GPS数据的时间戳更新为当前仿真时间
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.time import Time


class TimestampCorrector(Node):
    def __init__(self):
        super().__init__('timestamp_corrector')

        # 订阅原始GPS数据
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.fix_callback,
            10
        )

        # 发布修正后的GPS数据
        self.publisher = self.create_publisher(
            NavSatFix,
            '/fix_corrected',
            10
        )

        self.get_logger().info('GPS时间戳修正节点已启动')
        self.get_logger().info('订阅: /fix')
        self.get_logger().info('发布: /fix_corrected (时间戳已修正)')

        self.first_msg = True
        self.msg_count = 0

    def fix_callback(self, msg):
        # 创建新消息，复制所有数据
        corrected_msg = NavSatFix()
        corrected_msg.header.frame_id = msg.header.frame_id
        corrected_msg.status = msg.status
        corrected_msg.latitude = msg.latitude
        corrected_msg.longitude = msg.longitude
        corrected_msg.altitude = msg.altitude
        corrected_msg.position_covariance = msg.position_covariance
        corrected_msg.position_covariance_type = msg.position_covariance_type

        # 使用当前时间替换原始时间戳
        corrected_msg.header.stamp = self.get_clock().now().to_msg()

        # 发布修正后的消息
        self.publisher.publish(corrected_msg)

        self.msg_count += 1

        # 定期打印日志
        if self.first_msg or self.msg_count % 100 == 0:
            original_time = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
            corrected_time = Time.from_msg(corrected_msg.header.stamp).nanoseconds / 1e9
            time_diff = corrected_time - original_time

            self.get_logger().info(
                f'已处理 {self.msg_count} 条消息 | '
                f'原始时间戳: {original_time:.2f}s | '
                f'修正时间戳: {corrected_time:.2f}s | '
                f'时间差: {time_diff:.2f}s ({time_diff/86400:.1f}天)'
            )

            self.first_msg = False


def main(args=None):
    rclpy.init(args=args)
    node = TimestampCorrector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'节点关闭，共处理 {node.msg_count} 条GPS消息')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

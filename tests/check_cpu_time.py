#!/usr/bin/env python3
"""Debug script: ellenőrizd, hogy /lidar_filter/cpu_time_ms publikálódik-e."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class CPUTimeChecker(Node):
    def __init__(self):
        super().__init__('cpu_time_checker')
        self.count = 0
        self.sub = self.create_subscription(
            Float32,
            '/lidar_filter/cpu_time_ms',
            self.cb,
            10
        )
        self.get_logger().info('Hallgatom: /lidar_filter/cpu_time_ms...')

    def cb(self, msg: Float32):
        self.count += 1
        cpu_ms = float(msg.data)
        self.get_logger().info(f'[{self.count}] CPU: {cpu_ms:.2f} ms/frame')
        if self.count >= 10:
            self.get_logger().info(f'✅ Témakör működik! Legalább 10 üzenet érkezett.')
            rclpy.shutdown()


def main():
    rclpy.init()
    node = CPUTimeChecker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

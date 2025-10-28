#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class PathCreator(Node):
    def __init__(self):
        super().__init__('odom_path')

        # Declare parameters with default values
        self.declare_parameter('path_topic_name', '/path')
        self.declare_parameter('odom_topic_name', '/odom')
        self.declare_parameter('max_size', 1500)

        path_topic_name = self.get_parameter('path_topic_name').value
        odom_topic_name = self.get_parameter('odom_topic_name').value
        self.max_size = self.get_parameter('max_size').value

        # Initialize path and reset time
        self.path = Path()

        # Publisher and subscriber
        self.path_pub = self.create_publisher(Path, path_topic_name, 1)
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic_name,
            self.odom_cb,
            1
        )

        self.get_logger().info(
            f"Publishing path on [{path_topic_name}] from odometry [{odom_topic_name}]"
        )

    def odom_cb(self, msg: Odometry):
        if (len(self.path.poses) >= self.max_size) and (self.max_size > 0):
            del self.path.poses[0:int(self.max_size * 0.2)]

        # Append current pose
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.header = msg.header
        self.path.poses.append(pose)

        # Publish updated path
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = PathCreator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class Publisher_Node(Node):
    def __init__(self):
        super().__init__('publisher_node')

        self.publisher = self.create_publisher(String, "chatter", 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        message = "Hello, ROS2!"
        self.get_logger().info(message)
        self.get_logger().warn(message)
        self.get_logger().error(message)

        msg = String()
        msg.data = message
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # Node
    publisher_node = Publisher_Node()

    rclpy.spin(publisher_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
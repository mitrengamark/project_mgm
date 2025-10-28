import rclpy
from rclpy.node import Node

class Subscriber_Node(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        self.subscription = self.create_subscription()



def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber_Node()

    rclpy.spin(subscriber)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
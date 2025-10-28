import rclpy
from rclpy.node import Node
import random
import copy

from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker


class VizPub(Node):
    def __init__(self):
        super().__init__('test_viz')

        odom_topic_name = self.declare_parameter('odom_topic_name', '/odom').value
        self.sub = self.create_subscription(Odometry, odom_topic_name, self.callback_odom, 1)
        self.pub = self.create_publisher(MarkerArray, "/viz", 1)
        self.pose_list = []

    def callback_odom(self, msg: Odometry):
        marker_array = MarkerArray()

        marker = Marker()
        marker.header = msg.header
        marker.ns = "cube"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = msg.pose.pose

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.4

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
        
        marker_array.markers.append(marker)

        self.path_publisher(msg, marker_array)
        self.rectangle_publisher(marker_array)

        self.pub.publish(marker_array)

    def path_publisher(self, msg: Odometry, marker_array_: MarkerArray):
        self.pose_list.append(msg.pose.pose)
        
        marker_path = Marker()

        marker_path.header = msg.header

        marker_path.ns = "path"

        marker_path.type = Marker.SPHERE
        marker_path.action = Marker.ADD

        marker_path.scale.x = 0.1
        marker_path.scale.y = 0.1
        marker_path.scale.z = 0.1

        marker_path.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()

        for i in range(0, len(self.pose_list), 30):
            marker_path.id = i
            marker_path.pose = self.pose_list[i]

            marker_path.color.a = random.uniform(0.2, 1.0)
            marker_path.color.r = random.uniform(0.0, 1.0)
            marker_path.color.g = random.uniform(0.0, 1.0)
            marker_path.color.b = random.uniform(0.0, 1.0)

            marker_array_.markers.append(copy.deepcopy(marker_path))

    def rectangle_publisher(self, marker_array_: MarkerArray):
        x_min = 999.0
        x_max = -999.0
        y_min = 999.0
        y_max = -999.0

        for i in range(0,len(self.pose_list)-1):
            i_pose = self.pose_list[i]

            if (x_min > i_pose.position.x):
                x_min = i_pose.position.x

            if (x_max < i_pose.position.x):
                x_max = i_pose.position.x
    
    
            if (y_min > i_pose.position.y):
                y_min = i_pose.position.y

            if (y_max < i_pose.position.y):
                y_max = i_pose.position.y

        self.get_logger().info(f"x_min: {x_min} x_max: {x_max}")

        marker_rectangle = Marker()

        marker_rectangle.header.frame_id = "odom"
        marker_rectangle.header.stamp = self.get_clock().now().to_msg()

        marker_rectangle.ns = "rect"
        marker_rectangle.id = 0
        marker_rectangle.type = Marker.CUBE
        marker_rectangle.action = Marker.ADD

        marker_rectangle.color.a = 0.2
        marker_rectangle.color.r = 1.0
        marker_rectangle.color.b = 0.0
        marker_rectangle.color.g = 0.0

        marker_rectangle.pose.position.x = (x_min+x_max)/2.0
        marker_rectangle.pose.position.y = (y_min+y_max)/2.0
        marker_rectangle.pose.orientation.w = 1.0
        
        marker_rectangle.scale.x = abs(x_max - x_min)
        marker_rectangle.scale.y = abs(y_max - y_min)
        marker_rectangle.scale.z = 0.05

        marker_array_.markers.append(marker_rectangle)


def main(args=None):
    rclpy.init(args=args)
    viz_publisher = VizPub()
    rclpy.spin(viz_publisher)
    viz_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
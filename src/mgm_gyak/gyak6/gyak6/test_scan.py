import rclpy
from rclpy.node import Node
import copy
import math

from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2 as pcl2
from visualization_msgs.msg import MarkerArray, Marker

class ScanHandler(Node):
    def __init__(self):
        super().__init__('test_viz')

def main(args=None):
    rclpy.init(args=args)
    scan_handler_ = ScanHandler()
    rclpy.spin(scan_handler_)
    scan_handler_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
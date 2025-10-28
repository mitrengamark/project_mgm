

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import sys, select, termios, tty
from numpy import clip

control_keys = {
    'up'    : '\x41',
    'down'  : '\x42',
    'right' : '\x43',
    'left'  : '\x44',
    'space' : '\x20',
    'tab'   : '\x09'}

key_bindings = {
    '\x41' : ( 0.5 , 0.0),
    '\x42' : (-0.5 , 0.0),
    '\x43' : ( 0.0 ,-0.1),
    '\x44' : ( 0.0 , 0.1),
    '\x20' : ( 0.0 , 0.0),
    '\x09' : ( 0.0 , 0.0)}



class AckermannDriveKeyop(Node):
    def __init__(self):
        super().__init__('ackermann_drive_keyop_node')
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('topic', '/agent1/ackermann_cmd')
        self.declare_parameter('frame', 'agent1/base_link')

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.cmd_topic = self.get_parameter('topic').value
        self.frame = self.get_parameter('frame').value

        self.speed_range = [-float(self.max_speed), float(self.max_speed)]
        self.steering_angle_range = [-float(self.max_steering_angle), float(self.max_steering_angle)]
        for key in key_bindings:
            key_bindings[key] = (
                key_bindings[key][0] * float(self.max_speed) / 5,
                key_bindings[key][1] * float(self.max_steering_angle) / 5)

        self.speed = 0
        self.steering_angle = 0
        self.motors_pub = self.create_publisher(AckermannDriveStamped, self.cmd_topic, 1)
        self.timer = self.create_timer(1.0/5.0, self.pub_callback)
        self._shutdown = False
        self.print_state()
        try:
            self.key_loop()
        except Exception as e:
            self.get_logger().error(f"Exception in key_loop: {e}")
            self.finalize()

    def pub_callback(self):
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_cmd_msg.header.frame_id = self.frame
        ackermann_cmd_msg.drive.speed = self.speed
        ackermann_cmd_msg.drive.steering_angle = self.steering_angle
        self.motors_pub.publish(ackermann_cmd_msg)

    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        self.get_logger().info('*********************************************')
        self.get_logger().info('Use arrows to change speed and steering angle')
        self.get_logger().info('Use space to brake and tab to align wheels')
        self.get_logger().info('Press <ctrl-c> or <q> to exit')
        self.get_logger().info('*********************************************')
        self.get_logger().info(
            f'\033[34;1mSpeed: \033[32;1m{self.speed:.2f} m/s, '
            f'\033[34;1mSteer Angle: \033[32;1m{self.steering_angle:.2f} rad\033[0m')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def key_loop(self):
        self.settings = termios.tcgetattr(sys.stdin)
        while rclpy.ok() and not self._shutdown:
            key = self.get_key()
            if key in key_bindings.keys():
                if key == control_keys['space']:
                    self.speed = 0.0
                elif key == control_keys['tab']:
                    self.steering_angle = 0.0
                else:
                    self.speed = self.speed + key_bindings[key][0]
                    self.steering_angle = self.steering_angle + key_bindings[key][1]
                    self.speed = clip(self.speed, self.speed_range[0], self.speed_range[1])
                    self.steering_angle = clip(self.steering_angle, self.steering_angle_range[0], self.steering_angle_range[1])
                self.print_state()
            elif key == '\x03' or key == '\x71':  # ctrl-c or q
                self._shutdown = True
                rclpy.shutdown()
            else:
                continue

    def finalize(self):
        self.get_logger().info('Halting motors, aligning wheels and exiting...')
        try:
            self.settings = termios.tcgetattr(sys.stdin)
        except Exception:
            pass
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.speed = 0
        ackermann_cmd_msg.drive.steering_angle = 0
        self.motors_pub.publish(ackermann_cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    keyop = AckermannDriveKeyop()
    try:
        rclpy.spin(keyop)
    except KeyboardInterrupt:
        keyop.finalize()
    finally:
        keyop.finalize()
        keyop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

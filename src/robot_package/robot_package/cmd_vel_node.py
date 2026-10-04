"""Turn Nav2's velocity commands on 'cmd_vel' into per-wheel speeds on 'motor_pid_cmd'."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from robot_package.motor_pid_node import CMD_SPEED, CMD_STOP


class CmdVel(Node):
    """Subscribe to 'cmd_vel' (geometry_msgs/Twist) and publish "1 <left> <right>".

    Only linear.x (forward m/s) and angular.z (counter-clockwise rad/s) matter
    for a two-wheeled robot. Each wheel's speed is converted to encoder ticks/s
    for the Pico's PI loops. If a wheel would exceed 'max_tps', both wheels are
    scaled down together so the robot still follows the same curve, just slower.

    While Twists arrive the command is republished (motor_pid_driver stops the
    robot if it goes quiet for 1 s). Once they stop for 'timeout' seconds, one
    stop is sent and then nothing, so `keyboard_odom` can drive in between.
    """

    def __init__(self):
        super().__init__('cmd_vel_to_motor')

        self.declare_parameter('wheel_radius', 0.0335)    # m, same as odom
        self.declare_parameter('ticks_per_rev', 1960.0)   # encoder ticks per wheel turn, same as odom
        self.declare_parameter('wheel_base', 0.21)        # m, same as odom
        self.declare_parameter('max_tps', 3046)           # MAX_TARGET_TPS in the firmware
        self.declare_parameter('timeout', 0.5)            # s without cmd_vel before stopping
        wheel_radius = self.get_parameter('wheel_radius').value
        ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_tps = self.get_parameter('max_tps').value
        self.timeout = self.get_parameter('timeout').value

        self.ticks_per_meter = ticks_per_rev / (2.0 * math.pi * wheel_radius)

        self.publisher_ = self.create_publisher(String, 'motor_pid_cmd', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_twist, 10)
        self.create_timer(0.1, self.republish)

        self.line = None             # last command sent, None once quiet
        self.last_msg_time = None

        self.get_logger().info(
            f'cmd_vel -> motor_pid_cmd: {self.ticks_per_meter:.0f} ticks/m, '
            f'max {self.max_tps} ticks/s ({self.max_tps / self.ticks_per_meter:.2f} m/s), '
            f'wheel_base={self.wheel_base} m')

    def wheel_tps(self, v, w):
        """Signed (left, right) wheel speeds in ticks/s for v m/s and w rad/s."""
        left = (v - w * self.wheel_base / 2) * self.ticks_per_meter
        right = (v + w * self.wheel_base / 2) * self.ticks_per_meter
        fastest = max(abs(left), abs(right))
        if fastest > self.max_tps:
            left *= self.max_tps / fastest
            right *= self.max_tps / fastest
        return round(left), round(right)

    def on_twist(self, msg):
        left, right = self.wheel_tps(msg.linear.x, msg.angular.z)
        line = f'{CMD_SPEED} {left} {right}' if left or right else f'{CMD_STOP}'
        if line != self.line:
            self.get_logger().debug(f'motor_pid_cmd: {line}')
        self.line = line
        self.last_msg_time = self.get_clock().now()
        self.publisher_.publish(String(data=line))

    def republish(self):
        if self.line is None:
            return
        age = (self.get_clock().now() - self.last_msg_time).nanoseconds * 1e-9
        if age > self.timeout:
            # Nav2 stopped sending: stop once, then leave motor_pid_cmd to others
            self.publisher_.publish(String(data=f'{CMD_STOP}'))
            self.line = None
            return
        self.publisher_.publish(String(data=self.line))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVel()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publisher_.publish(String(data=f'{CMD_STOP}'))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

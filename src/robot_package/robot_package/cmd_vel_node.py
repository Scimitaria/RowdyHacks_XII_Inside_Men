"""Turn Nav2's velocity commands on 'cmd_vel' into per-wheel duty on 'motor_cmd'."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from robot_package.motor_node import CMD_STOP, CMD_WHEELS


class CmdVel(Node):
    """Subscribe to 'cmd_vel' (geometry_msgs/Twist) and publish "7 <left> <right>".

    Only linear.x (forward m/s) and angular.z (counter-clockwise rad/s) matter
    for a two-wheeled robot. The motors are open loop, so speed maps straight
    to duty: 'max_wheel_speed' is the wheel speed at 100 % duty.

    While Twists arrive the command is republished (motor_driver stops the
    robot if it goes quiet for 1 s). Once they stop for 'timeout' seconds, one
    stop is sent and then nothing, so `keyboard` can drive in between.
    """

    def __init__(self):
        super().__init__('cmd_vel_to_motor')

        self.declare_parameter('max_wheel_speed', 0.5)   # m/s at 100 % duty (calibrate)
        self.declare_parameter('min_duty', 30)           # wheels don't turn below this
        self.declare_parameter('wheel_base', 0.21)       # m, same as odom
        self.declare_parameter('timeout', 0.5)           # s without cmd_vel before stopping
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.min_duty = self.get_parameter('min_duty').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.timeout = self.get_parameter('timeout').value

        self.publisher_ = self.create_publisher(String, 'motor_cmd', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_twist, 10)
        self.create_timer(0.1, self.republish)

        self.line = None             # last command sent, None once quiet
        self.last_msg_time = None

        self.get_logger().info(
            f'cmd_vel -> motor_cmd: max_wheel_speed={self.max_wheel_speed} m/s, '
            f'min_duty={self.min_duty}, wheel_base={self.wheel_base} m')

    def duty(self, speed):
        """Signed duty (-100-100) for a wheel speed in m/s."""
        duty = round(100 * speed / self.max_wheel_speed)
        if duty == 0:
            return 0
        magnitude = min(max(abs(duty), self.min_duty), 100)
        return magnitude if duty > 0 else -magnitude

    def on_twist(self, msg):
        v, w = msg.linear.x, msg.angular.z
        left = self.duty(v - w * self.wheel_base / 2)
        right = self.duty(v + w * self.wheel_base / 2)
        line = f'{CMD_WHEELS} {left} {right}' if left or right else f'{CMD_STOP}'
        if line != self.line:
            self.get_logger().debug(f'motor_cmd: {line}')
        self.line = line
        self.last_msg_time = self.get_clock().now()
        self.publisher_.publish(String(data=line))

    def republish(self):
        if self.line is None:
            return
        age = (self.get_clock().now() - self.last_msg_time).nanoseconds * 1e-9
        if age > self.timeout:
            # Nav2 stopped sending: stop once, then leave motor_cmd to others
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

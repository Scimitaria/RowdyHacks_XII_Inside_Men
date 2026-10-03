"""Poll the Pico's wheel encoders and publish differential-drive odometry."""

import math

import rclpy
import serial
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# Encoder query understood by the Pico firmware; it replies
# "ENC <count_a> <count_b> <time_us>" (motor A = left, motor B = right).
ENCODER_QUERY = b'6\n'

# Variance for fields this node doesn't measure (z, roll, pitch, vy, ...).
UNKNOWN_VARIANCE = 1e6


class WheelOdometry(Node):
    """Publish wheel odometry on '/wheel/odom' from the Pico's encoder counts."""

    def __init__(self):
        super().__init__('wheel_odometry')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('ticks_per_rev', 1440.0)   # counts per wheel turn, all 4 edges
        self.declare_parameter('wheel_radius', 0.033)     # m
        self.declare_parameter('wheel_base', 0.16)        # m, distance between wheel centers
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', False)       # leave off when the EKF is running
        self.declare_parameter('velocity_variance', 0.01)
        self.declare_parameter('yaw_rate_variance', 0.05)

        port = self.get_parameter('port').value
        rate = self.get_parameter('rate').value
        ticks_per_rev = self.get_parameter('ticks_per_rev').value
        wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.velocity_variance = self.get_parameter('velocity_variance').value
        self.yaw_rate_variance = self.get_parameter('yaw_rate_variance').value

        self.meters_per_tick = 2.0 * math.pi * wheel_radius / ticks_per_rev

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last = None   # (count_a, count_b, time_us) from the previous reply
        self.rx_buffer = b''

        # motor_node opens the same port; Linux allows both, and only this
        # node reads, so it also drains the Pico's "OK ..." replies.
        self.serial = serial.Serial(port, 115200, timeout=0, write_timeout=0.5)

        self.publisher_ = self.create_publisher(Odometry, 'wheel/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_timer(1.0 / rate, self.poll)

        self.get_logger().info(
            f'Wheel odometry from {port}: ticks_per_rev={ticks_per_rev}, '
            f'wheel_radius={wheel_radius} m, wheel_base={self.wheel_base} m')

    def poll(self):
        """Handle replies to the previous query, then send the next one."""
        try:
            if self.serial.in_waiting:
                self.rx_buffer += self.serial.read(self.serial.in_waiting)
            self.serial.write(ENCODER_QUERY)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial error: {e}', throttle_duration_sec=2.0)
            return

        *lines, self.rx_buffer = self.rx_buffer.split(b'\n')
        for line in lines:
            parts = line.decode(errors='ignore').split()
            if len(parts) == 4 and parts[0] == 'ENC':
                try:
                    self.on_encoders(int(parts[1]), int(parts[2]), int(parts[3]))
                except ValueError:
                    pass

    def on_encoders(self, count_a, count_b, time_us):
        last, self.last = self.last, (count_a, count_b, time_us)
        if last is None:
            return
        dt = (time_us - last[2]) * 1e-6
        if dt <= 0.0:   # duplicate reply, or the Pico rebooted
            return

        d_left = (count_a - last[0]) * self.meters_per_tick
        d_right = (count_b - last[1]) * self.meters_per_tick
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        # Integrate along the mid-step heading.
        heading = self.theta + d_theta / 2.0
        self.x += d_center * math.cos(heading)
        self.y += d_center * math.sin(heading)
        self.theta = math.atan2(math.sin(self.theta + d_theta),
                                math.cos(self.theta + d_theta))

        self.publish(d_center / dt, d_theta / dt)

    def publish(self, v, w):
        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # The EKF requires non-zero covariances; diagonal order is
        # x, y, z, roll, pitch, yaw (or vx, vy, vz, vroll, vpitch, vyaw).
        pose_diag = [0.05, 0.05, UNKNOWN_VARIANCE,
                     UNKNOWN_VARIANCE, UNKNOWN_VARIANCE, 0.1]
        twist_diag = [self.velocity_variance, self.velocity_variance, UNKNOWN_VARIANCE,
                      UNKNOWN_VARIANCE, UNKNOWN_VARIANCE, self.yaw_rate_variance]
        for i in range(6):
            odom.pose.covariance[i * 7] = pose_diag[i]
            odom.twist.covariance[i * 7] = twist_diag[i]
        self.publisher_.publish(odom)

        if self.tf_broadcaster:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.frame_id
            tf.child_frame_id = self.child_frame_id
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

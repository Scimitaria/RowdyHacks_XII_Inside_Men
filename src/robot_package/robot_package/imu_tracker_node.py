"""Track rotation (yaw) in degrees from an IMU topic."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


class ImuTracker(Node):
    """Publish the IMU's heading in degrees on 'rotation_degrees'."""

    def __init__(self):
        super().__init__('imu_tracker')

        self.declare_parameter('imu_topic', '/camera/imu')
        self.declare_parameter('use_orientation', False)
        imu_topic = self.get_parameter('imu_topic').value
        self.use_orientation = self.get_parameter('use_orientation').value

        self.yaw = 0.0
        self.start_yaw = None
        self.last_time = None

        self.create_subscription(Imu, imu_topic, self.on_imu, 50)
        self.publisher_ = self.create_publisher(Float64, 'rotation_degrees', 10)
        self.create_timer(1.0, self.log_rotation)

        mode = 'orientation' if self.use_orientation else 'gyro integration'
        self.get_logger().info(f'Tracking rotation from {imu_topic} using {mode}')

    def on_imu(self, msg):
        if self.use_orientation:
            q = msg.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            if self.start_yaw is None:
                self.start_yaw = yaw
            self.yaw = yaw - self.start_yaw
        else:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self.last_time is not None:
                dt = t - self.last_time
                if 0.0 < dt < 0.5:
                    self.yaw += msg.angular_velocity.z * dt
            self.last_time = t

        out = Float64()
        out.data = math.degrees(self.yaw)
        self.publisher_.publish(out)

    def log_rotation(self):
        self.get_logger().info(f'Rotation: {math.degrees(self.yaw):.1f} deg')


def main(args=None):
    rclpy.init(args=args)
    node = ImuTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Convert the IMU heading in degrees ('rotation_degrees') into sensor_msgs/Imu."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

# Roll/pitch are not measured; a huge variance tells the EKF to ignore them.
UNKNOWN_VARIANCE = 1e6


class ImuAdapter(Node):
    """Publish the yaw from 'rotation_degrees' as an Imu on 'imu/data'."""

    def __init__(self):
        super().__init__('imu_adapter')

        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('yaw_variance', 0.01)
        self.frame_id = self.get_parameter('frame_id').value
        self.yaw_variance = self.get_parameter('yaw_variance').value

        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.create_subscription(Float64, 'rotation_degrees', self.on_rotation, 10)
        self.get_logger().info('Converting rotation_degrees -> imu/data')

    def on_rotation(self, msg):
        yaw = math.radians(msg.data)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.frame_id
        # Rotation about Z only.
        imu.orientation.z = math.sin(yaw / 2.0)
        imu.orientation.w = math.cos(yaw / 2.0)
        imu.orientation_covariance = [
            UNKNOWN_VARIANCE, 0.0, 0.0,
            0.0, UNKNOWN_VARIANCE, 0.0,
            0.0, 0.0, self.yaw_variance,
        ]
        # -1 in the first element means "this field is not provided".
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0
        self.publisher_.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = ImuAdapter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

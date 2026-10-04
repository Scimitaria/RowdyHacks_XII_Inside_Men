"""Track rotation (yaw) in degrees from the OAK-D camera's built-in IMU."""

import math

import depthai as dai
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from robot_package.imu_yaw import YawTracker, create_imu_queue, yaw_to_imu


class ImuTracker(Node):
    """Publish the IMU's heading as a sensor_msgs/Imu on 'imu/data' (read by the EKF)."""

    def __init__(self):
        super().__init__('imu_tracker')

        self.declare_parameter('use_orientation', False)
        self.declare_parameter('imu_rate', 200)
        # The yaw is about the robot's vertical axis, so it belongs to base_link.
        self.declare_parameter('imu_frame_id', 'base_link')
        self.declare_parameter('yaw_variance', 0.01)
        self.use_orientation = self.get_parameter('use_orientation').value
        imu_rate = self.get_parameter('imu_rate').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self.yaw_variance = self.get_parameter('yaw_variance').value

        self.tracker = YawTracker(self.use_orientation, self.get_logger().info)

        self.pipeline = dai.Pipeline()
        self.imu_queue = create_imu_queue(
            self.pipeline, self.use_orientation, imu_rate)
        self.pipeline.start()

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.create_timer(0.005, self.poll_imu)
        self.create_timer(1.0, self.log_rotation)

        mode = 'orientation' if self.use_orientation else 'gyro integration'
        self.get_logger().info(f'Tracking rotation from OAK-D IMU using {mode}')
        if not self.use_orientation:
            self.get_logger().info('Calibrating gyro, keep the robot still...')

    def poll_imu(self):
        for imu_data in self.imu_queue.tryGetAll():
            for packet in imu_data.packets:
                yaw = self.tracker.update(packet)
                if yaw is not None:
                    self.imu_pub.publish(yaw_to_imu(
                        yaw, self.get_clock().now().to_msg(),
                        self.imu_frame_id, self.yaw_variance))

    def log_rotation(self):
        self.get_logger().info(
            f'Rotation: {math.degrees(self.tracker.yaw):.1f} deg')

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


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

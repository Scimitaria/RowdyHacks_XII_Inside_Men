"""Track rotation (yaw) in degrees from the OAK-D camera's built-in IMU."""

import math

import depthai as dai
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from robot_package.imu_yaw import YawTracker, create_imu_queue


class ImuTracker(Node):
    """Publish the IMU's heading in degrees on 'rotation_degrees'."""

    def __init__(self):
        super().__init__('imu_tracker')

        self.declare_parameter('use_orientation', False)
        self.declare_parameter('imu_rate', 200)
        self.use_orientation = self.get_parameter('use_orientation').value
        imu_rate = self.get_parameter('imu_rate').value

        self.tracker = YawTracker(self.use_orientation, self.get_logger().info)

        self.pipeline = dai.Pipeline()
        self.imu_queue = create_imu_queue(
            self.pipeline, self.use_orientation, imu_rate)
        self.pipeline.start()

        self.publisher_ = self.create_publisher(Float64, 'rotation_degrees', 10)
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
                    out = Float64()
                    out.data = math.degrees(yaw)
                    self.publisher_.publish(out)

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

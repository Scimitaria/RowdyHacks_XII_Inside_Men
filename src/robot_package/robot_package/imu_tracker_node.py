"""Track rotation (yaw) in degrees from the OAK-D camera's built-in IMU."""

import math

import depthai as dai
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# Samples averaged at startup to find the gravity direction and gyro bias.
CALIBRATION_SAMPLES = 200


class ImuTracker(Node):
    """Publish the IMU's heading in degrees on 'rotation_degrees'."""

    def __init__(self):
        super().__init__('imu_tracker')

        self.declare_parameter('use_orientation', False)
        self.declare_parameter('imu_rate', 200)
        self.use_orientation = self.get_parameter('use_orientation').value
        imu_rate = self.get_parameter('imu_rate').value

        self.yaw = 0.0
        self.start_yaw = None
        self.last_time = None

        # Gyro mode: 'up' is the unit gravity vector in the IMU frame, so
        # yaw rate = gyro . up no matter how the camera is mounted.
        self.up = None
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.accel_sum = [0.0, 0.0, 0.0]
        self.gyro_sum = [0.0, 0.0, 0.0]
        self.calib_count = 0

        self.pipeline = dai.Pipeline()
        imu = self.pipeline.create(dai.node.IMU)
        if self.use_orientation:
            # Fused orientation; only available on BNO086-equipped cameras.
            imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, imu_rate)
        else:
            imu.enableIMUSensor(
                [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW],
                imu_rate)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        self.imu_queue = imu.out.createOutputQueue(maxSize=50, blocking=False)
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
                self.on_packet(packet)

    def on_packet(self, packet):
        if self.use_orientation:
            q = packet.rotationVector
            yaw = math.atan2(2.0 * (q.real * q.k + q.i * q.j),
                             1.0 - 2.0 * (q.j * q.j + q.k * q.k))
            if self.start_yaw is None:
                self.start_yaw = yaw
            self.yaw = yaw - self.start_yaw
        else:
            a = packet.acceleroMeter
            g = packet.gyroscope
            if self.up is None:
                self.calibrate(a, g)
                return

            t = g.getTimestampDevice().total_seconds()
            if self.last_time is not None:
                dt = t - self.last_time
                if 0.0 < dt < 0.5:
                    gyro = (g.x, g.y, g.z)
                    rate = sum((w - b) * u for w, b, u
                               in zip(gyro, self.gyro_bias, self.up))
                    self.yaw += rate * dt
            self.last_time = t

        out = Float64()
        out.data = math.degrees(self.yaw)
        self.publisher_.publish(out)

    def calibrate(self, a, g):
        for i, (av, gv) in enumerate(zip((a.x, a.y, a.z), (g.x, g.y, g.z))):
            self.accel_sum[i] += av
            self.gyro_sum[i] += gv
        self.calib_count += 1
        if self.calib_count < CALIBRATION_SAMPLES:
            return

        norm = math.sqrt(sum(v * v for v in self.accel_sum))
        self.up = [v / norm for v in self.accel_sum]
        self.gyro_bias = [v / self.calib_count for v in self.gyro_sum]
        self.get_logger().info(
            f'Calibrated: up=({self.up[0]:.2f}, {self.up[1]:.2f}, '
            f'{self.up[2]:.2f}), gyro bias={self.gyro_bias}')

    def log_rotation(self):
        self.get_logger().info(f'Rotation: {math.degrees(self.yaw):.1f} deg')

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

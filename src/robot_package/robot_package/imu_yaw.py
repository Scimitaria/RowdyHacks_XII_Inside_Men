"""Shared OAK-D IMU setup and yaw tracking, used by imu_tracker and oak_camera."""

import math

import depthai as dai

# Samples averaged at startup to find the gravity direction and gyro bias.
CALIBRATION_SAMPLES = 200


def create_imu_queue(pipeline, use_orientation, imu_rate):
    """Add an IMU node to 'pipeline' and return its output queue."""
    imu = pipeline.create(dai.node.IMU)
    if use_orientation:
        # Fused orientation; only available on BNO086-equipped cameras.
        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, imu_rate)
    else:
        imu.enableIMUSensor(
            [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW],
            imu_rate)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)
    return imu.out.createOutputQueue(maxSize=50, blocking=False)


class YawTracker:
    """Turn IMU packets into a yaw angle (radians) relative to the start."""

    def __init__(self, use_orientation=False, log=None):
        self.use_orientation = use_orientation
        self.log = log or (lambda text: None)
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

    def update(self, packet):
        """Feed one IMU packet; return the yaw in radians, or None while calibrating."""
        if self.use_orientation:
            q = packet.rotationVector
            yaw = math.atan2(2.0 * (q.real * q.k + q.i * q.j),
                             1.0 - 2.0 * (q.j * q.j + q.k * q.k))
            if self.start_yaw is None:
                self.start_yaw = yaw
            self.yaw = yaw - self.start_yaw
            return self.yaw

        a = packet.acceleroMeter
        g = packet.gyroscope
        if self.up is None:
            self.calibrate(a, g)
            return None

        t = g.getTimestampDevice().total_seconds()
        if self.last_time is not None:
            dt = t - self.last_time
            if 0.0 < dt < 0.5:
                gyro = (g.x, g.y, g.z)
                rate = sum((w - b) * u for w, b, u
                           in zip(gyro, self.gyro_bias, self.up))
                self.yaw += rate * dt
        self.last_time = t
        return self.yaw

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
        self.log(f'Calibrated: up=({self.up[0]:.2f}, {self.up[1]:.2f}, '
                 f'{self.up[2]:.2f}), gyro bias={self.gyro_bias}')

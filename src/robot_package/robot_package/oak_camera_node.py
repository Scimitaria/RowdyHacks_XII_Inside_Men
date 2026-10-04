"""Publish RGB, aligned depth, camera calibration and the IMU heading from the OAK-D Pro W.

RTAB-Map can subscribe to this node directly; no feeder node is needed.
"""

import time
from datetime import timedelta

import depthai as dai
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu

from robot_package.imu_yaw import YawTracker, create_imu_queue, yaw_to_imu


class OakCamera(Node):
    """Publish 'camera/rgb/image_raw' (bgr8), 'camera/depth/image_raw' (16UC1, mm),
    'camera/rgb/camera_info', and the IMU heading on 'imu/data' (sensor_msgs/Imu,
    read by the EKF) - the same topic imu_tracker publishes.

    RGB and depth are paired on the camera, share one timestamp (the capture
    time), have the same size, and depth is aligned to the RGB camera, so each
    depth pixel lines up with the RGB pixel at the same position.

    The camera can only be opened by one process, so this node also owns the
    IMU; don't run imu_tracker alongside it.
    """

    def __init__(self):
        super().__init__('oak_camera')

        self.declare_parameter('fps', 15)
        # 640x400 matches the Pro W sensors' 16:10 shape, so nothing is
        # cropped and RGB and depth come out the same size.
        self.declare_parameter('rgb_width', 640)
        self.declare_parameter('rgb_height', 400)
        self.declare_parameter('publish_rgb', True)
        self.declare_parameter('publish_depth', True)
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('use_orientation', False)
        self.declare_parameter('imu_rate', 200)
        # The yaw is about the robot's vertical axis, so it belongs to base_link.
        self.declare_parameter('imu_frame_id', 'base_link')
        self.declare_parameter('yaw_variance', 0.01)
        # Images must use an optical frame (z forward, x right, y down).
        self.declare_parameter('frame_id', 'oak_camera_optical')
        self.declare_parameter('sync_threshold_ms', 20)
        fps = self.get_parameter('fps').value
        self.size = (self.get_parameter('rgb_width').value,
                     self.get_parameter('rgb_height').value)
        publish_rgb = self.get_parameter('publish_rgb').value
        publish_depth = self.get_parameter('publish_depth').value
        publish_imu = self.get_parameter('publish_imu').value
        self.frame_id = self.get_parameter('frame_id').value

        self.image_queue = None
        self.imu_queue = None
        self.counts = {'rgb': 0, 'depth': 0}
        self.size_checked = False

        self.pipeline = dai.Pipeline()
        outputs = {}
        if publish_rgb:
            cam = self.pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_A)
            outputs['rgb'] = cam.requestOutput(
                self.size, dai.ImgFrame.Type.BGR888p, fps=fps)
        if publish_depth:
            left = self.pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_B)
            right = self.pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_C)
            stereo = self.pipeline.create(dai.node.StereoDepth).build(
                left.requestOutput((640, 400), fps=fps),
                right.requestOutput((640, 400), fps=fps))
            # Line depth up with the RGB camera so pixels match.
            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            stereo.setOutputSize(*self.size)
            outputs['depth'] = stereo.depth

        self.stream_names = list(outputs)
        if len(outputs) == 2:
            # Pair RGB and depth frames taken at the same moment on the camera.
            sync = self.pipeline.create(dai.node.Sync)
            sync.setSyncThreshold(timedelta(
                milliseconds=self.get_parameter('sync_threshold_ms').value))
            for name, out in outputs.items():
                out.link(sync.inputs[name])
            self.image_queue = sync.out.createOutputQueue(maxSize=4, blocking=False)
        elif outputs:
            self.image_queue = next(iter(outputs.values())).createOutputQueue(
                maxSize=4, blocking=False)

        if publish_imu:
            use_orientation = self.get_parameter('use_orientation').value
            self.tracker = YawTracker(use_orientation, self.get_logger().info)
            self.imu_queue = create_imu_queue(
                self.pipeline, use_orientation,
                self.get_parameter('imu_rate').value)
        self.pipeline.start()

        if publish_rgb:
            self.rgb_pub = self.create_publisher(Image, 'camera/rgb/image_raw', 10)
            self.info_pub = self.create_publisher(
                CameraInfo, 'camera/rgb/camera_info', 10)
            self.camera_info = self.read_camera_info()
        if publish_depth:
            self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        if publish_imu:
            self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
            self.imu_frame_id = self.get_parameter('imu_frame_id').value
            self.yaw_variance = self.get_parameter('yaw_variance').value
            self.get_logger().info('Calibrating gyro, keep the robot still...')
        self.create_timer(0.005, self.poll_camera)
        self.create_timer(1.0, self.log_rates)

        self.get_logger().info(
            f'OAK-D streaming {self.size[0]}x{self.size[1]} at {fps} fps '
            f'(rgb={publish_rgb}, depth={publish_depth}, imu={publish_imu}), '
            f'frame_id={self.frame_id}')

    def read_camera_info(self):
        """Build the RGB CameraInfo from the camera's factory calibration."""
        width, height = self.size
        calib = self.pipeline.getDefaultDevice().readCalibration()
        k = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, width, height)
        d = calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)
        fx, cx = k[0][0], k[0][2]
        fy, cy = k[1][1], k[1][2]

        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width, info.height = width, height
        # DepthAI stores 14 coefficients; the first 8 are the
        # rational_polynomial model (k1, k2, p1, p2, k3, k4, k5, k6).
        info.distortion_model = 'rational_polynomial'
        info.d = [float(x) for x in d[:8]]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.get_logger().info(
            f'RGB intrinsics: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}')
        return info

    def capture_stamp(self, frame):
        """ROS time at which the frame was captured, not when it arrived.

        getTimestamp() is the capture time on the host's monotonic clock, so
        the gap to time.monotonic() is the camera-to-host latency.
        """
        latency = time.monotonic() - frame.getTimestamp().total_seconds()
        latency = min(max(latency, 0.0), 0.5)  # guard against clock oddities
        return (self.get_clock().now() - Duration(seconds=latency)).to_msg()

    def poll_camera(self):
        if self.image_queue is not None:
            for msg in self.image_queue.tryGetAll():
                if len(self.stream_names) == 2:
                    frames = {name: msg[name] for name in self.stream_names}
                else:
                    frames = {self.stream_names[0]: msg}
                self.publish_frames(frames)
        if self.imu_queue is not None:
            for imu_data in self.imu_queue.tryGetAll():
                for packet in imu_data.packets:
                    yaw = self.tracker.update(packet)
                    if yaw is not None:
                        self.imu_pub.publish(yaw_to_imu(
                            yaw, self.get_clock().now().to_msg(),
                            self.imu_frame_id, self.yaw_variance))

    def publish_frames(self, frames):
        # One stamp for the whole pair, taken from the RGB frame if present.
        stamp = self.capture_stamp(frames.get('rgb', frames.get('depth')))
        if 'rgb' in frames:
            self.rgb_pub.publish(
                self.to_image(frames['rgb'].getCvFrame(), 'bgr8', stamp))
            self.camera_info.header.stamp = stamp
            self.info_pub.publish(self.camera_info)
            self.counts['rgb'] += 1
        if 'depth' in frames:
            depth = frames['depth'].getFrame()
            self.check_size_once(depth)
            self.depth_pub.publish(self.to_image(depth, '16UC1', stamp))
            self.counts['depth'] += 1

    def check_size_once(self, depth):
        if self.size_checked:
            return
        self.size_checked = True
        height, width = depth.shape[:2]
        if (width, height) != self.size:
            self.get_logger().warn(
                f'Depth is {width}x{height} but RGB is {self.size[0]}x'
                f'{self.size[1]}; RTAB-Map needs them to match.')

    def to_image(self, array, encoding, stamp):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = array.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = array.strides[0]
        msg.data = array.tobytes()
        return msg

    def log_rates(self):
        self.get_logger().info(
            f"Published last second: rgb={self.counts['rgb']}, "
            f"depth={self.counts['depth']}")
        self.counts = {'rgb': 0, 'depth': 0}

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OakCamera()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
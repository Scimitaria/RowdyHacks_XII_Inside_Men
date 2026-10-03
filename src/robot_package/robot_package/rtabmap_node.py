"""Prepare OAK-D images, odometry and TF for RTAB-Map, which builds the map."""

import cv2
import message_filters
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


class RtabmapFeeder(Node):
    """Pair RGB and depth frames and publish them for RTAB-Map.

    Subscribes to 'camera/rgb/image_raw' and 'camera/depth/image_raw' (from
    oak_camera), 'odometry/filtered' (nav_msgs/Odometry) and TF. The images are paired by
    timestamp and republished with one shared stamp, the same size, and a
    CameraInfo, on 'rtabmap_input/...'. Frames are held back until odometry
    is arriving and TF links the odom, base and camera frames, so RTAB-Map
    never sees an image it cannot place.
    """

    def __init__(self):
        super().__init__('rtabmap_feeder')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('odom_topic', 'odometry/filtered')
        self.declare_parameter('sync_slop', 0.05)
        self.declare_parameter('odom_timeout', 1.0)
        # Pinhole intrinsics of the RGB image. Defaults are a rough guess for
        # a 640x480 OAK-D; replace them with the camera's calibrated values.
        self.declare_parameter('fx', 450.0)
        self.declare_parameter('fy', 450.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_timeout = Duration(
            seconds=self.get_parameter('odom_timeout').value)
        self.intrinsics = [self.get_parameter(k).value
                           for k in ('fx', 'fy', 'cx', 'cy')]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_odom_time = None
        self.sent_count = 0
        self.dropped_count = 0
        self.drop_reason = ''

        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        rgb_sub = message_filters.Subscriber(self, Image, 'camera/rgb/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, 'camera/depth/image_raw')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], 10, self.get_parameter('sync_slop').value)
        self.sync.registerCallback(self.on_images)

        self.rgb_pub = self.create_publisher(Image, 'rtabmap_input/rgb/image', 10)
        self.depth_pub = self.create_publisher(Image, 'rtabmap_input/depth/image', 10)
        self.info_pub = self.create_publisher(
            CameraInfo, 'rtabmap_input/rgb/camera_info', 10)
        self.create_timer(2.0, self.log_status)

        self.get_logger().info(
            f'Feeding RTAB-Map: odom frame={self.odom_frame}, '
            f'base frame={self.base_frame}')

    def on_odom(self, msg):
        self.last_odom_time = self.get_clock().now()

    def on_images(self, rgb, depth):
        reason = self.not_ready_reason(rgb.header.frame_id)
        if reason:
            self.dropped_count += 1
            self.drop_reason = reason
            return

        if (depth.height, depth.width) != (rgb.height, rgb.width):
            depth = self.resize_depth(depth, rgb.width, rgb.height)
        # The camera stamps each stream separately; RTAB-Map needs one stamp.
        depth.header.stamp = rgb.header.stamp
        depth.header.frame_id = rgb.header.frame_id

        self.rgb_pub.publish(rgb)
        self.depth_pub.publish(depth)
        self.info_pub.publish(self.make_camera_info(rgb))
        self.sent_count += 1

    def not_ready_reason(self, camera_frame):
        """Return why frames can't be mapped yet, or '' if they can."""
        now = self.get_clock().now()
        if self.last_odom_time is None:
            return 'no odometry messages yet'
        if now - self.last_odom_time > self.odom_timeout:
            return 'odometry has stopped publishing'
        # Time() asks for the latest transform; RTAB-Map does the exact lookup.
        if not self.tf_buffer.can_transform(self.odom_frame, self.base_frame, Time()):
            return f'no TF {self.odom_frame} -> {self.base_frame}'
        if not self.tf_buffer.can_transform(self.base_frame, camera_frame, Time()):
            return f'no TF {self.base_frame} -> {camera_frame}'
        return ''

    @staticmethod
    def resize_depth(msg, width, height):
        # Nearest neighbour: blending depths would invent distances.
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        resized = cv2.resize(depth, (width, height), interpolation=cv2.INTER_NEAREST)
        out = Image()
        out.header = msg.header
        out.height, out.width = height, width
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = resized.strides[0]
        out.data = resized.tobytes()
        return out

    def make_camera_info(self, rgb):
        fx, fy, cx, cy = self.intrinsics
        info = CameraInfo()
        info.header = rgb.header
        info.width, info.height = rgb.width, rgb.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5  # the OAK-D publishes rectified images
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def log_status(self):
        msg = f'Frames sent to RTAB-Map: {self.sent_count}, held back: {self.dropped_count}'
        if self.dropped_count and not self.sent_count:
            msg += f' ({self.drop_reason})'
        self.get_logger().info(msg)
        self.sent_count = 0
        self.dropped_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = RtabmapFeeder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

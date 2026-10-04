"""Republish the OAK-D RGB stream as JPEG, small enough to stream over Tailscale."""

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class ImageRelay(Node):
    """Subscribe to 'camera/rgb/image_raw' and publish JPEGs on 'camera/rgb/image_relay/compressed'.

    JPEG encodes a 640x400 frame in ~2 ms on the Pi and cuts it from 750 KB to
    tens of KB. Frames go out best-effort with a queue of 1: over a slow link
    a lost frame is skipped instead of resent, so the stream stays live rather
    than falling behind. Subscribers must be best-effort too (in RViz, set the
    Image display's Reliability Policy to Best Effort).

    The '/compressed' suffix is the image_transport convention, so RViz and
    rqt_image_view can show it as the 'compressed' transport of
    'camera/rgb/image_relay'.

    Topics are parameters, e.g.:
        ros2 run robot_package image_relay --ros-args -p jpeg_quality:=50 -p scale:=0.5
    """

    def __init__(self):
        super().__init__('image_relay')

        self.declare_parameter('input_topic', 'camera/rgb/image_raw')
        self.declare_parameter('output_topic', 'camera/rgb/image_relay/compressed')
        self.declare_parameter('jpeg_quality', 70)   # 0-100: lower = smaller and blurrier
        self.declare_parameter('scale', 1.0)         # resize before encoding, e.g. 0.5 = half size
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.scale = self.get_parameter('scale').value

        self.publisher_ = self.create_publisher(CompressedImage, output_topic, qos_profile_sensor_data)
        # Queue of 1: if encoding falls behind, drop old frames instead of queueing them
        self.create_subscription(Image, input_topic, self.on_image, 1)

        self.get_logger().info(
            f'Relaying {input_topic} -> {output_topic} '
            f'(JPEG quality {self.jpeg_quality}, scale {self.scale})')

    def on_image(self, msg):
        if msg.encoding not in ('bgr8', 'rgb8'):
            self.get_logger().warn(f'Unsupported encoding {msg.encoding!r}, expected bgr8',
                                   throttle_duration_sec=5.0)
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step // 3, 3)
        frame = frame[:, :msg.width]
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if self.scale != 1.0:
            frame = cv2.resize(frame, None, fx=self.scale, fy=self.scale,
                               interpolation=cv2.INTER_AREA)

        ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = 'bgr8; jpeg compressed bgr8'  # what image_transport's compressed plugin expects
        out.data = jpeg.tobytes()
        self.publisher_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImageRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

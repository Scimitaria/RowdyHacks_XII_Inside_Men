"""Show the OAK-D RGB and depth streams in OpenCV windows."""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraViewer(Node):
    """Subscribe to 'camera/rgb/image_raw' and 'camera/depth/image_raw' and display them."""

    def __init__(self):
        super().__init__('camera_viewer')

        self.declare_parameter('show_rgb', True)
        self.declare_parameter('show_depth', True)
        self.declare_parameter('max_depth_mm', 5000)
        self.max_depth_mm = self.get_parameter('max_depth_mm').value

        if self.get_parameter('show_rgb').value:
            self.create_subscription(Image, 'camera/rgb/image_raw', self.on_rgb, 10)
        if self.get_parameter('show_depth').value:
            self.create_subscription(Image, 'camera/depth/image_raw', self.on_depth, 10)

        self.get_logger().info('Waiting for camera frames (press q in a window to quit)')

    def on_rgb(self, msg):
        frame = self.to_array(msg, np.uint8, 3)
        cv2.imshow('rgb', frame)
        self.poll_keys()

    def on_depth(self, msg):
        depth = self.to_array(msg, np.uint16, 1)
        # Clip to max_depth_mm and scale to 8 bits so near = bright colors.
        scaled = np.clip(depth, 0, self.max_depth_mm) * (255.0 / self.max_depth_mm)
        colored = cv2.applyColorMap(scaled.astype(np.uint8), cv2.COLORMAP_JET)
        colored[depth == 0] = 0  # no depth reading -> black
        cv2.imshow('depth', colored)
        self.poll_keys()

    @staticmethod
    def to_array(msg, dtype, channels):
        array = np.frombuffer(msg.data, dtype=dtype)
        shape = (msg.height, msg.width, channels) if channels > 1 else (msg.height, msg.width)
        return array.reshape(shape)

    def poll_keys(self):
        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

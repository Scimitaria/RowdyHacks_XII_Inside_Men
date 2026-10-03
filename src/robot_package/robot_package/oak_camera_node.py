"""Publish RGB and depth images from the OAK-D Pro W."""

import depthai as dai
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class OakCamera(Node):
    """Publish 'camera/rgb/image_raw' (bgr8) and 'camera/depth/image_raw' (16UC1, mm)."""

    def __init__(self):
        super().__init__('oak_camera')

        self.declare_parameter('fps', 15)
        self.declare_parameter('rgb_width', 640)
        self.declare_parameter('rgb_height', 480)
        self.declare_parameter('publish_rgb', True)
        self.declare_parameter('publish_depth', True)
        self.declare_parameter('frame_id', 'oak_camera')
        fps = self.get_parameter('fps').value
        rgb_size = (self.get_parameter('rgb_width').value,
                    self.get_parameter('rgb_height').value)
        self.publish_rgb = self.get_parameter('publish_rgb').value
        self.publish_depth = self.get_parameter('publish_depth').value
        self.frame_id = self.get_parameter('frame_id').value

        self.rgb_queue = None
        self.depth_queue = None
        self.rgb_count = 0
        self.depth_count = 0

        self.pipeline = dai.Pipeline()
        if self.publish_rgb:
            cam = self.pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_A)
            self.rgb_queue = cam.requestOutput(
                rgb_size, dai.ImgFrame.Type.BGR888p, fps=fps
            ).createOutputQueue(maxSize=4, blocking=False)
        if self.publish_depth:
            left = self.pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_B)
            right = self.pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_C)
            stereo = self.pipeline.create(dai.node.StereoDepth).build(
                left.requestOutput((640, 400), fps=fps),
                right.requestOutput((640, 400), fps=fps))
            # Line depth up with the RGB camera so pixels match.
            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            self.depth_queue = stereo.depth.createOutputQueue(
                maxSize=4, blocking=False)
        self.pipeline.start()

        if self.publish_rgb:
            self.rgb_pub = self.create_publisher(Image, 'camera/rgb/image_raw', 10)
        if self.publish_depth:
            self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.create_timer(0.005, self.poll_camera)
        self.create_timer(1.0, self.log_rates)

        self.get_logger().info(
            f'OAK-D streaming at {fps} fps (rgb={self.publish_rgb}, '
            f'depth={self.publish_depth})')

    def poll_camera(self):
        if self.rgb_queue is not None:
            for frame in self.rgb_queue.tryGetAll():
                self.rgb_pub.publish(self.to_image(frame.getCvFrame(), 'bgr8'))
                self.rgb_count += 1
        if self.depth_queue is not None:
            for frame in self.depth_queue.tryGetAll():
                self.depth_pub.publish(self.to_image(frame.getFrame(), '16UC1'))
                self.depth_count += 1

    def to_image(self, array, encoding):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = array.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = array.strides[0]
        msg.data = array.tobytes()
        return msg

    def log_rates(self):
        self.get_logger().info(
            f'Published last second: rgb={self.rgb_count}, depth={self.depth_count}')
        self.rgb_count = 0
        self.depth_count = 0

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

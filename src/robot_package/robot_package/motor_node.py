import os,select,sys,termios,threading,tty,rclpy,serial
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

# Terminal escape sequences for arrow keys
KEY_UP = '\x1b[A'
KEY_DOWN = '\x1b[B'
KEY_RIGHT = '\x1b[C'
KEY_LEFT = '\x1b[D'
KEY_ESC = '\x1b'
KEY_SPACE = ' '


def read_key(fd):
    """Block until a key is pressed and return it, including arrow sequences."""
    key = os.read(fd, 1).decode(errors='ignore')
    if key == KEY_ESC:
        # Arrow keys send ESC followed by '[' and a letter. A lone ESC has
        # nothing following it, so wait briefly to tell the two apart.
        ready, _, _ = select.select([fd], [], [], 0.05)
        if ready:
            key += os.read(fd, 2).decode(errors='ignore')
    return key

class KeyboardToUSB(Node):
    def __init__(self):
        super().__init__('keyboard_to_usb')

        self.publisher_ = self.create_publisher(String, 'usb_data', 10)
        # write_timeout: if the Pico stops reading, fail instead of freezing the key loop
        self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1, write_timeout=0.5)
        self.create_subscription(Image, 'camera/depth/image_raw', self.on_depth, 10)

        self.duty_cycle = 50
        self.cmd = 0
        self.nearest_mm = None

        self.create_timer(1.0, self.log)

        self.get_logger().info(
            "\n============================================\n"
            "Keyboard to USB Node Started!\n"
            "Use UP arrow to send 1, duty_cycle.\n"
            "Use DOWN arrow to send 2, duty_cycle.\n"
            "Use LEFT arrow to send 3, duty_cycle.\n"
            "Use RIGHT arrow to send 4, duty_cycle.\n"
            "Use SPACEBAR to stop (5).\n"
            "Press ESC to exit.\n"
            "============================================\n"
        )

    def on_press(self, key):
        """Handle a key press. Returns False when the node should exit."""
        if key == KEY_UP:
            self.cmd = 1
            self.publish_usb()
        if key == KEY_DOWN:
            self.cmd = 2
            self.publish_usb()
        if key == KEY_LEFT:
            self.cmd = 3
            self.publish_usb()
        if key == KEY_RIGHT:
            self.cmd = 4
            self.publish_usb()
        if key == KEY_SPACE:
            self.cmd = 5
            self.publish_usb()
        if key == KEY_ESC:
            self.cmd = 5
            self.publish_usb()
            self.get_logger().info("Exiting keyboard listener...")
            return False
        return True

    def on_depth(self, msg):
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        # Only look at the middle of the frame so the floor and edges don't count
        h, w = depth.shape
        roi = depth[h // 3 : 2 * h // 3, w // 4 : 3 * w // 4]
        valid = roi[roi > 0]  # 0 = no depth reading
        # 1st percentile instead of min() so one noisy pixel doesn't set the distance
        self.nearest_mm = float(np.percentile(valid, 1)) if valid.size else None

        if self.nearest_mm and (self.nearest_mm/1000) < 1.0:
            self.cmd = 3
            self.publish_usb()
        elif self.nearest_mm and (self.nearest_mm/1000) >= 1.0:
            self.cmd = 1
            self.publish_usb()

    def log(self):
        if self.nearest_mm is None:
            self.get_logger().info("Nearest object: no depth data")
        else:
            self.get_logger().info(f"Nearest object: {self.nearest_mm / 1000:.2f} m")

    def publish_usb(self):
        msg = String()
        msg.data = f"{self.cmd} {self.duty_cycle}"
        self.publisher_.publish(msg)                     # optional: keeps it visible on the ROS topic
        self.get_logger().info(f"Published USB_data: {msg.data}")
        try:
            self.serial.write((msg.data + '\n').encode())    # this is what actually reaches the Pico
        except serial.SerialTimeoutException:
            self.get_logger().error("Serial write timed out: is the Pico running the firmware?")

class AlgorithmToUSB(Node):
    def __init__(self):
        super().__init__('algorithm_to_usb')

        self.publisher_ = self.create_publisher(String, 'usb_data', 10)
        # write_timeout: if the Pico stops reading, fail instead of freezing the key loop
        self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1, write_timeout=0.5)

        self.duty_cycle = 75
        self.cmd = 0

        self.get_logger().info(
            "\n============================================\n"
            "Algorithm to USB Node Started!\n"
            "============================================\n"
        )

    def run(self):
        while True:
            self.cmd = 1
            self.publish_usb()

    def publish_usb(self):
        msg = String()
        msg.data = f"{self.cmd} {self.duty_cycle}"
        self.publisher_.publish(msg)                     # optional: keeps it visible on the ROS topic
        self.get_logger().info(f"Published USB_data: {msg.data}")
        try:
            self.serial.write((msg.data + '\n').encode())    # this is what actually reaches the Pico
        except serial.SerialTimeoutException:
            self.get_logger().error("Serial write timed out: is the Pico running the firmware?")


def main(args=None):
    if not sys.stdin.isatty():
        print("keyboard node needs an interactive terminal (use `ros2 run`, not a launch file)", file=sys.stderr)
        return

    rclpy.init(args=args)
    if True:
        node = KeyboardToUSB()
    else: 
        node = AlgorithmToUSB()

    # Spin in the background so timers and subscriptions fire while the main thread blocks on keys
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    if isinstance(node, AlgorithmToUSB):
        node.run()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        # cbreak: deliver keys immediately without echo, but keep Ctrl+C working
        tty.setcbreak(fd)
        while rclpy.ok() and node.on_press(read_key(fd)): pass
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        # Stop the motor on exit
        node.cmd = 5
        node.publish_usb()
        node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

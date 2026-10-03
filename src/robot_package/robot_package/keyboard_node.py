import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# Terminal escape sequences for arrow keys
KEY_UP = '\x1b[A'
KEY_DOWN = '\x1b[B'
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


class KeyboardToPWM(Node):

    def __init__(self):
        super().__init__('keyboard_to_pwm')

        # Create a publisher on the 'pwm_output' topic
        self.publisher_ = self.create_publisher(Int32, 'pwm_output', 10)

        # Initialize PWM values (assuming standard 0 to 255 range)
        self.pwm_value = 0
        self.step = 5
        self.max_pwm = 255
        self.min_pwm = 0

        self.get_logger().info(
            "\n============================================\n"
            "Keyboard to PWM Node Started!\n"
            "Use UP arrow to increase PWM.\n"
            "Use DOWN arrow to decrease PWM.\n"
            "Use SPACEBAR to stop (0 PWM).\n"
            "Press ESC to exit.\n"
            "============================================\n"
        )

    def on_press(self, key):
        """Handle a key press. Returns False when the node should exit."""
        if key == KEY_UP:
            self.pwm_value = min(self.pwm_value + self.step, self.max_pwm)
            self.publish_pwm()
        elif key == KEY_DOWN:
            self.pwm_value = max(self.pwm_value - self.step, self.min_pwm)
            self.publish_pwm()
        elif key == KEY_SPACE:
            self.pwm_value = 0
            self.publish_pwm()
        elif key == KEY_ESC:
            self.get_logger().info("Exiting keyboard listener...")
            return False
        return True

    def publish_pwm(self):
        msg = Int32()
        msg.data = self.pwm_value
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published PWM: {msg.data}")


def main(args=None):
    if not sys.stdin.isatty():
        print("keyboard node needs an interactive terminal (use `ros2 run`, not a launch file)",
              file=sys.stderr)
        return

    rclpy.init(args=args)
    node = KeyboardToPWM()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        # cbreak: deliver keys immediately without echo, but keep Ctrl+C working
        tty.setcbreak(fd)
        while rclpy.ok() and node.on_press(read_key(fd)):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        # Stop the motor on exit
        node.pwm_value = 0
        node.publish_pwm()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

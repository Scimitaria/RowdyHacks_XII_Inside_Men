"""Forward motor commands from the 'motor_cmd' topic to the Pico over serial."""

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

# Commands understood by the Pico firmware, one per line: "<cmd> [duty]",
# duty 0-100. 6 (read encoders) is left to the odom node.
CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT, CMD_STOP = 1, 2, 3, 4, 5
# "7 <left> <right>": signed per-wheel duty -100-100, negative is backward
CMD_WHEELS = 7
STOP_LINE = f'{CMD_STOP}'


class MotorDriver(Node):
    """Subscribe to 'motor_cmd' (std_msgs/String, "<cmd> [duty]") and drive the Pico.

    Controllers must keep republishing their command (a few times a second):
    if nothing arrives for 'timeout' seconds the motors are stopped, so a
    crashed or closed controller can't leave the robot driving.
    """

    def __init__(self):
        super().__init__('motor_driver')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('timeout', 1.0)   # s without a command before stopping
        port = self.get_parameter('port').value
        self.timeout = self.get_parameter('timeout').value

        # odom opens the same port; Linux allows both, and only odom reads,
        # so it also drains the Pico's replies to these commands.
        self.serial = serial.Serial(port, 115200, timeout=0.1, write_timeout=0.5)

        self.current = None          # last line written to the Pico
        self.last_msg_time = None
        self.write(STOP_LINE)

        self.create_subscription(String, 'motor_cmd', self.on_cmd, 10)
        self.create_timer(0.1, self.watchdog)

        self.get_logger().info(
            f'Motor driver on {port}: listening on /motor_cmd ("<cmd> [duty]", '
            f'1 fwd, 2 back, 3 left, 4 right, 5 stop, "7 <left> <right>" per wheel), '
            f'timeout {self.timeout} s')

    def on_cmd(self, msg):
        line = self.parse(msg.data)
        if line is None:
            self.get_logger().warn(f'Ignoring bad motor_cmd: {msg.data!r}',
                                   throttle_duration_sec=2.0)
            return
        self.last_msg_time = self.get_clock().now()
        self.write(line)

    @staticmethod
    def parse(text):
        """Return the line to send to the Pico, or None if 'text' isn't a valid command."""
        parts = text.split()
        try:
            values = [int(p) for p in parts]
        except ValueError:
            return None
        if values and values[0] == CMD_WHEELS:
            if len(values) != 3 or not all(-100 <= v <= 100 for v in values[1:]):
                return None
            return ' '.join(str(v) for v in values)
        if not 1 <= len(values) <= 2 or not CMD_FORWARD <= values[0] <= CMD_STOP:
            return None
        if values[0] in (CMD_FORWARD, CMD_BACKWARD) and len(values) < 2:
            return None                       # the Pico needs a duty for these
        if len(values) == 2 and not 0 <= values[1] <= 100:
            return None
        if values[0] == CMD_STOP:
            return STOP_LINE                  # duty means nothing for stop
        return ' '.join(str(v) for v in values)

    def watchdog(self):
        if self.current == STOP_LINE or self.last_msg_time is None:
            return
        age = (self.get_clock().now() - self.last_msg_time).nanoseconds * 1e-9
        if age > self.timeout:
            self.get_logger().warn(f'No motor_cmd for {age:.1f} s, stopping')
            self.write(STOP_LINE)

    def write(self, line):
        # The Pico holds the last command, so only write when it changes
        if line == self.current:
            return
        try:
            self.serial.write((line + '\n').encode())
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial write failed ({e}): is the Pico running the firmware?',
                                    throttle_duration_sec=2.0)
            self.current = None               # unknown state: retry on the next command
            return
        self.current = line
        self.get_logger().info(f'Sent to Pico: {line}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Stop the motors on exit
        node.current = None
        node.write(STOP_LINE)
        node.serial.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

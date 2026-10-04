"""Forward wheel speed commands from the 'motor_pid_cmd' topic to the Pico over serial.

For the pi_pico_PID firmware, which runs a PI speed loop per wheel.
"""

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

# Commands understood by the pi_pico_PID firmware, one per line:
# "1 <left> <right>": signed per-wheel speed in encoder ticks/s, positive is
# forward, negative backward, 0 coasts that wheel. 6 (read encoders) is left
# to the odom node.
CMD_SPEED, CMD_STOP = 1, 5
STOP_LINE = f'{CMD_STOP}'


class MotorPidDriver(Node):
    """Subscribe to 'motor_pid_cmd' (std_msgs/String, "1 <left> <right>" or "5") and drive the Pico.

    Controllers must keep republishing their command (a few times a second):
    if nothing arrives for 'timeout' seconds the motors are stopped, so a
    crashed or closed controller can't leave the robot driving.
    """

    def __init__(self):
        super().__init__('motor_pid_driver')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('timeout', 1.0)       # s without a command before stopping
        self.declare_parameter('max_tps', 3046)      # same as MAX_TARGET_TPS in the firmware
        port = self.get_parameter('port').value
        self.timeout = self.get_parameter('timeout').value
        self.max_tps = self.get_parameter('max_tps').value

        # odom opens the same port; Linux allows both, and only odom reads,
        # so it also drains the Pico's replies to these commands.
        self.serial = serial.Serial(port, 115200, timeout=0.1, write_timeout=0.5)

        self.current = None          # last line written to the Pico
        self.last_msg_time = None
        self.write(STOP_LINE)

        self.create_subscription(String, 'motor_pid_cmd', self.on_cmd, 10)
        self.create_timer(0.1, self.watchdog)

        self.get_logger().info(
            f'Motor PID driver on {port}: listening on /motor_pid_cmd '
            f'("1 <left> <right>" ticks/s, +-{self.max_tps}, or "5" stop), '
            f'timeout {self.timeout} s')

    def on_cmd(self, msg):
        line = self.parse(msg.data, self.max_tps)
        if line is None:
            self.get_logger().warn(f'Ignoring bad motor_pid_cmd: {msg.data!r}',
                                   throttle_duration_sec=2.0)
            return
        self.last_msg_time = self.get_clock().now()
        self.write(line)

    @staticmethod
    def parse(text, max_tps):
        """Return the line to send to the Pico, or None if 'text' isn't a valid command."""
        parts = text.split()
        try:
            values = [int(p) for p in parts]
        except ValueError:
            return None
        if values == [CMD_STOP]:
            return STOP_LINE
        if len(values) != 3 or values[0] != CMD_SPEED:
            return None
        if not all(-max_tps <= v <= max_tps for v in values[1:]):
            return None                       # the Pico would reject it too
        if values[1] == 0 and values[2] == 0:
            return STOP_LINE                  # same effect, and resets the PI loops
        return ' '.join(str(v) for v in values)

    def watchdog(self):
        if self.current == STOP_LINE or self.last_msg_time is None:
            return
        age = (self.get_clock().now() - self.last_msg_time).nanoseconds * 1e-9
        if age > self.timeout:
            self.get_logger().warn(f'No motor_pid_cmd for {age:.1f} s, stopping')
            self.write(STOP_LINE)

    def write(self, line):
        # The Pico holds the last command, so only write when it changes
        if line == self.current:
            return
        try:
            self.serial.write((line + '\n').encode())
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial write failed ({e}): is the Pico running the PID firmware?',
                                    throttle_duration_sec=2.0)
            self.current = None               # unknown state: retry on the next command
            return
        self.current = line
        self.get_logger().info(f'Sent to Pico: {line}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorPidDriver()
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

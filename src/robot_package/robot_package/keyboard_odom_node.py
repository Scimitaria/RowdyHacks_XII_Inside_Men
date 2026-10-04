"""Drive the robot with the arrow keys by publishing wheel speeds on 'motor_pid_cmd'.

Like keyboard_node, but for the pi_pico_PID firmware: commands are per-wheel
speeds in encoder ticks/s ("1 <left> <right>") instead of duty cycles.
"""

import os
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from robot_package.keyboard_node import (DISABLE_KEY_EVENTS, DRIVE_KEYS, FOCUS_OUT, KEY_BACKSPACE,
                                         KEY_DOWN, KEY_ESC, KEY_LEFT, KEY_RIGHT, KEY_SPACE, KEY_UP,
                                         PERIOD, PRESS, RELEASE, TURN_KEYS, enable_key_events,
                                         read_event)
from robot_package.motor_pid_node import CMD_SPEED, CMD_STOP


class KeyboardPidTeleop(Node):
    """Publish the arrow-key command on 'motor_pid_cmd', repeated so motor_pid_driver keeps it.

    UP/DOWN drive, ramping from 'ramp_start' to 'speed' (ticks/s). LEFT/RIGHT
    spin in place at 'turn_speed', or curve while driving (outer wheel 'speed',
    inner wheel 'arc_inner_speed'). Letting go of every key stops the robot.

    Shares the robot with explore_node. If the explorer is running at startup
    it keeps control and this node publishes nothing on 'motor_pid_cmd'. SPACE
    takes over: it publishes True on the latched 'manual_control' topic, so the
    explorer cancels its Nav2 goal, and stops the robot. BACKSPACE stops the
    robot and hands it back (False). Exiting while in control stops the robot
    but leaves the explorer paused until a keyboard hands it back.

    With 'key_events' (the terminal supports the kitty keyboard protocol) the
    node sees real key releases. Otherwise the terminal only sends presses and
    auto-repeats the last key held, so all keys count as released once nothing
    arrives for 'hold_timeout', and letting go of LEFT/RIGHT while still holding
    UP/DOWN stops the robot.
    """

    def __init__(self, key_events):
        super().__init__('keyboard_pid_teleop')

        # Speeds in encoder ticks/s; keep them under MAX_TARGET_TPS in the firmware.
        self.declare_parameter('speed', 2000)            # top forward/backward speed, outer wheel
        self.declare_parameter('ramp_start', 800)        # speed the ramp starts from
        self.declare_parameter('ramp_time', 0.5)         # s from ramp_start to speed
        self.declare_parameter('turn_speed', 1000)       # each wheel when spinning in place
        self.declare_parameter('arc_inner_speed', 900)   # inner wheel while curving
        self.declare_parameter('hold_timeout', 0.6)      # s; must exceed the key auto-repeat delay
        self.top_speed = self.get_parameter('speed').value
        self.ramp_start = self.get_parameter('ramp_start').value
        self.ramp_time = self.get_parameter('ramp_time').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.arc_inner_speed = self.get_parameter('arc_inner_speed').value
        self.hold_timeout = self.get_parameter('hold_timeout').value

        self.key_events = key_events
        # on_key runs on the main thread, the timer on the spin thread
        self.lock = threading.Lock()
        self.held = set()       # arrow keys currently held
        self.key_seen = 0.0     # time.monotonic() of the last arrow-key press
        self.drive = 0          # +1 forward, -1 backward, 0 not driving
        self.turn = 0           # +1 left, -1 right, 0 straight
        self.speed = 0.0        # current ramped speed, ticks/s
        self.logged_mode = None

        self.publisher_ = self.create_publisher(String, 'motor_pid_cmd', 10)
        # Latched, so an explorer started later still learns who is driving
        self.manual_pub = self.create_publisher(
            Bool, 'manual_control', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Start with the explorer in control if it's running. Otherwise drive
        # right away, and say so in case the explorer was just missed.
        self.manual = not self.explorer_running()
        if self.manual:
            self.manual_pub.publish(Bool(data=True))
        # motor_pid_driver stops the robot if commands stop arriving, so keep sending
        self.create_timer(PERIOD, self.update)

        if key_events:
            keys = "Terminal reports key releases: hold any combination of arrows\n"
        else:
            keys = ("Terminal can't report key releases (try kitty, WezTerm, Ghostty or\n"
                    "Alacritty): hold UP/DOWN first, then LEFT/RIGHT to curve\n")
        self.get_logger().info(
            "\n============================================\n"
            "Keyboard PID teleop started (publishing /motor_pid_cmd, ticks/s)\n"
            f"Hold UP/DOWN: forward/backward, ramping {self.ramp_start} -> {self.top_speed}\n"
            f"Hold LEFT/RIGHT: spin at {self.turn_speed}\n"
            f"Both: curve ({self.top_speed}/{self.arc_inner_speed})\n"
            "Release keys or SPACE: stop, ESC: stop and exit\n"
            "SPACE while exploring: stop the explorer and take over\n"
            "BACKSPACE: stop and hand control back to the explorer\n"
            + keys +
            "============================================\n"
        )
        self.log_control()

    def explorer_running(self, wait=1.0):
        """Whether explore_node shows up in the ROS graph within 'wait' seconds."""
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            if 'explorer' in self.get_node_names():
                return True
            time.sleep(0.1)
        return False

    def log_control(self):
        if self.manual:
            self.get_logger().info("Keyboard in control (BACKSPACE: hand back to the explorer)")
        else:
            self.get_logger().info("Explorer in control (SPACE: stop it and take over)")

    def set_manual(self, manual):
        """Take control from the explorer (True) or hand it back (False). Call with the lock held."""
        self.held.clear()
        if not manual:
            # Stop before going quiet; Nav2 takes over from standstill
            self.publisher_.publish(String(data=f"{CMD_STOP}"))
        self.manual = manual
        self.logged_mode = None
        self.manual_pub.publish(Bool(data=manual))
        self.log_control()

    def on_key(self, key, event):
        """Handle a key event. Returns False when the node should exit."""
        with self.lock:
            if key == KEY_BACKSPACE:
                if event != RELEASE and self.manual:
                    self.set_manual(False)
                return True
            if key in DRIVE_KEYS or key in TURN_KEYS:
                if not self.manual:
                    if event != RELEASE:
                        self.get_logger().info("Explorer in control: press SPACE to take over",
                                               throttle_duration_sec=2.0)
                    return True
                if event == RELEASE:
                    self.held.discard(key)
                else:
                    if not self.key_events:
                        # Only the newest key repeats, so a press replaces the keys it
                        # conflicts with; a turn key keeps the drive key for a curve.
                        if key in DRIVE_KEYS:
                            self.held.clear()
                        else:
                            self.held -= TURN_KEYS.keys()
                    self.held.add(key)
                    self.key_seen = time.monotonic()
            elif key in (KEY_SPACE, KEY_ESC, FOCUS_OUT) and event != RELEASE:
                self.held.clear()
                if key == KEY_SPACE and not self.manual:
                    self.set_manual(True)
            else:
                return True
        self.update(ramp=False)
        if key == KEY_ESC:
            self.get_logger().info("Exiting keyboard teleop...")
            return False
        return True

    def update(self, ramp=True):
        with self.lock:
            if not self.manual:
                return                          # leave motor_pid_cmd to Nav2
            if not self.key_events and time.monotonic() - self.key_seen > self.hold_timeout:
                self.held.clear()               # no key repeating: all released
            drive = (KEY_UP in self.held) - (KEY_DOWN in self.held)
            turn = (KEY_LEFT in self.held) - (KEY_RIGHT in self.held)
            if drive != self.drive:
                self.speed = self.ramp_start    # starting or reversing: ramp from low
            elif drive and ramp:
                step = (self.top_speed - self.ramp_start) * PERIOD / self.ramp_time
                self.speed = min(self.speed + step, self.top_speed)
            self.drive, self.turn = drive, turn
            line = self.line()
        self.publisher_.publish(String(data=line))
        if (drive, turn) != self.logged_mode:
            self.logged_mode = (drive, turn)
            self.get_logger().info(f"motor_pid_cmd: {line}")

    def line(self):
        if not self.drive:
            if self.turn:
                # Spin in place: positive turn (left) runs the left wheel backward
                s = self.turn_speed * self.turn
                return f"{CMD_SPEED} {-s} {s}"
            return f"{CMD_STOP}"
        if not self.turn:
            s = round(self.speed) * self.drive
            return f"{CMD_SPEED} {s} {s}"
        # Curve: the wheel on the pressed side is the slower, inner one
        outer, inner = self.top_speed, self.arc_inner_speed
        left, right = (inner, outer) if self.turn > 0 else (outer, inner)
        return f"{CMD_SPEED} {left * self.drive} {right * self.drive}"


def main(args=None):
    if not sys.stdin.isatty():
        print("keyboard_odom node needs an interactive terminal (use `ros2 run`, not a launch file)",
              file=sys.stderr)
        return

    rclpy.init(args=args)
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    key_events = False
    node = None
    try:
        # cbreak: deliver keys immediately without echo, but keep Ctrl+C working
        tty.setcbreak(fd)
        key_events = enable_key_events(fd)
        node = KeyboardPidTeleop(key_events)
        # Spin in the background so the repeat timer fires while the main thread blocks on keys
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        while rclpy.ok() and node.on_key(*read_event(fd)):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        if key_events:
            os.write(sys.stdout.fileno(), DISABLE_KEY_EVENTS)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        if node is not None:
            # Stop on exit. After Ctrl+C rclpy is already shut down and can't publish;
            # then motor_pid_driver's timeout stops the robot instead.
            if rclpy.ok() and node.manual:
                node.on_key(KEY_SPACE, PRESS)
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

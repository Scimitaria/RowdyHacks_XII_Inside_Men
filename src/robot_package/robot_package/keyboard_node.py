"""Drive the robot with the arrow keys by publishing on 'motor_cmd'."""

import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robot_package.motor_node import (CMD_BACKWARD, CMD_FORWARD, CMD_LEFT, CMD_RIGHT, CMD_STOP,
                                      CMD_WHEELS)

KEY_UP, KEY_DOWN, KEY_RIGHT, KEY_LEFT = 'up', 'down', 'right', 'left'
KEY_ESC = '\x1b'
KEY_SPACE = ' '
FOCUS_OUT = 'focus-out'      # the terminal window lost focus (keys can't be tracked)
REPLY_FLAGS = 'reply-flags'  # terminal answered the kitty keyboard protocol query
REPLY_DA = 'reply-da'        # terminal answered the device attributes query

# Final byte of the arrow key escape sequences ESC [ A .. ESC [ D
ARROWS = {'A': KEY_UP, 'B': KEY_DOWN, 'C': KEY_RIGHT, 'D': KEY_LEFT}

# Key event types, numbered as in the kitty keyboard protocol
PRESS, REPEAT, RELEASE = 1, 2, 3

# +1 forward / left, -1 backward / right
DRIVE_KEYS = {KEY_UP: 1, KEY_DOWN: -1}
TURN_KEYS = {KEY_LEFT: 1, KEY_RIGHT: -1}

PERIOD = 0.05   # s between published commands (also the ramp step)

# Kitty keyboard protocol (https://sw.kovidgoyal.net/kitty/keyboard-protocol/):
# query support, then push flag 2 (report repeat and release events). 1004
# turns on focus in/out reports so keys held while switching windows get dropped.
QUERY_KEY_EVENTS = b'\x1b[?u\x1b[c'
ENABLE_KEY_EVENTS = b'\x1b[>2u\x1b[?1004h'
DISABLE_KEY_EVENTS = b'\x1b[<u\x1b[?1004l'


def read_byte(fd, timeout=None):
    """Read one character, or return '' if none arrives within 'timeout' seconds."""
    if timeout is not None:
        ready, _, _ = select.select([fd], [], [], timeout)
        if not ready:
            return ''
    return os.read(fd, 1).decode(errors='ignore')


def read_event(fd):
    """Block until a key event arrives and return (key, event type).

    Arrows arrive as 'ESC [ A', or with key events on as 'ESC [ 1;<mods>:<event> A'.
    Unknown sequences return key None.
    """
    ch = read_byte(fd)
    if ch != KEY_ESC:
        return ch, PRESS
    # A lone ESC has nothing following it, so wait briefly to tell it from a sequence
    if read_byte(fd, 0.05) != '[':
        return KEY_ESC, PRESS
    params = ''
    while True:
        ch = read_byte(fd, 0.05)
        if not ch:
            return None, PRESS          # cut off
        if '\x40' <= ch <= '\x7e':      # final byte of the sequence
            break
        params += ch

    if params.startswith('?'):
        return {'u': REPLY_FLAGS, 'c': REPLY_DA}.get(ch), PRESS
    if ch == 'O' and not params:
        return FOCUS_OUT, PRESS

    fields = params.split(';')
    event = PRESS
    if len(fields) > 1 and ':' in fields[1]:
        try:
            event = int(fields[1].split(':')[1])
        except ValueError:
            pass
    if ch in ARROWS:
        return ARROWS[ch], event
    if ch == 'u':
        code = fields[0].split(':')[0]
        return {'27': KEY_ESC, '32': KEY_SPACE}.get(code), event
    return None, event


def enable_key_events(fd):
    """Turn on key release reports if the terminal supports them; return whether it does."""
    out = sys.stdout.fileno()
    os.write(out, QUERY_KEY_EVENTS)
    # Every terminal answers the device attributes query (ESC [ c), so its reply
    # marks the end. Only terminals with the kitty protocol answer ESC [ ? u first.
    supported = False
    deadline = time.monotonic() + 0.5
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([fd], [], [], remaining)[0]:
            break
        key, _ = read_event(fd)
        if key == REPLY_FLAGS:
            supported = True
        elif key == REPLY_DA:
            break
    if supported:
        os.write(out, ENABLE_KEY_EVENTS)
    return supported


class KeyboardTeleop(Node):
    """Publish the arrow-key command on 'motor_cmd', repeated so motor_driver keeps it.

    UP/DOWN drive, ramping quickly from 'ramp_start' to 'duty_cycle'. LEFT/RIGHT
    spin in place, or curve while driving (outer wheel 'duty_cycle', inner wheel
    'arc_inner_duty'). Letting go of every key stops the robot.

    With 'key_events' (the terminal supports the kitty keyboard protocol) the
    node sees real key releases. Otherwise the terminal only sends presses and
    auto-repeats the last key held, so all keys count as released once nothing
    arrives for 'hold_timeout', and letting go of LEFT/RIGHT while still holding
    UP/DOWN stops the robot.
    """

    def __init__(self, key_events):
        super().__init__('keyboard_teleop')

        self.declare_parameter('duty_cycle', 70)       # top forward/backward speed, outer wheel
        self.declare_parameter('ramp_start', 50)       # duty the ramp starts from
        self.declare_parameter('ramp_time', 0.3)       # s from ramp_start to duty_cycle
        self.declare_parameter('turn_duty', 50)        # spin in place
        self.declare_parameter('arc_inner_duty', 30)   # inner wheel while curving
        self.declare_parameter('hold_timeout', 0.6)    # s; must exceed the key auto-repeat delay
        self.duty_cycle = self.get_parameter('duty_cycle').value
        self.ramp_start = self.get_parameter('ramp_start').value
        self.ramp_time = self.get_parameter('ramp_time').value
        self.turn_duty = self.get_parameter('turn_duty').value
        self.arc_inner_duty = self.get_parameter('arc_inner_duty').value
        self.hold_timeout = self.get_parameter('hold_timeout').value

        self.key_events = key_events
        # on_key runs on the main thread, the timer on the spin thread
        self.lock = threading.Lock()
        self.held = set()       # arrow keys currently held
        self.key_seen = 0.0     # time.monotonic() of the last arrow-key press
        self.drive = 0          # +1 forward, -1 backward, 0 not driving
        self.turn = 0           # +1 left, -1 right, 0 straight
        self.speed = 0.0        # current ramped duty
        self.logged_mode = None

        self.publisher_ = self.create_publisher(String, 'motor_cmd', 10)
        # motor_driver stops the robot if commands stop arriving, so keep sending
        self.create_timer(PERIOD, self.update)

        if key_events:
            keys = "Terminal reports key releases: hold any combination of arrows\n"
        else:
            keys = ("Terminal can't report key releases (try kitty, WezTerm, Ghostty or\n"
                    "Alacritty): hold UP/DOWN first, then LEFT/RIGHT to curve\n")
        self.get_logger().info(
            "\n============================================\n"
            "Keyboard teleop started (publishing /motor_cmd)\n"
            f"Hold UP/DOWN: forward/backward, ramping {self.ramp_start} -> {self.duty_cycle}\n"
            f"Hold LEFT/RIGHT: spin at {self.turn_duty}\n"
            f"Both: curve ({self.duty_cycle}/{self.arc_inner_duty})\n"
            "Release keys or SPACE: stop, ESC: stop and exit\n"
            + keys +
            "============================================\n"
        )

    def on_key(self, key, event):
        """Handle a key event. Returns False when the node should exit."""
        with self.lock:
            if key in DRIVE_KEYS or key in TURN_KEYS:
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
            else:
                return True
        self.update(ramp=False)
        if key == KEY_ESC:
            self.get_logger().info("Exiting keyboard teleop...")
            return False
        return True

    def update(self, ramp=True):
        with self.lock:
            if not self.key_events and time.monotonic() - self.key_seen > self.hold_timeout:
                self.held.clear()               # no key repeating: all released
            drive = (KEY_UP in self.held) - (KEY_DOWN in self.held)
            turn = (KEY_LEFT in self.held) - (KEY_RIGHT in self.held)
            if drive != self.drive:
                self.speed = self.ramp_start    # starting or reversing: ramp from low
            elif drive and ramp:
                step = (self.duty_cycle - self.ramp_start) * PERIOD / self.ramp_time
                self.speed = min(self.speed + step, self.duty_cycle)
            self.drive, self.turn = drive, turn
            line = self.line()
        self.publisher_.publish(String(data=line))
        if (drive, turn) != self.logged_mode:
            self.logged_mode = (drive, turn)
            self.get_logger().info(f"motor_cmd: {line}")

    def line(self):
        if not self.drive:
            if self.turn:
                return f"{CMD_LEFT if self.turn > 0 else CMD_RIGHT} {self.turn_duty}"
            return f"{CMD_STOP}"
        if not self.turn:
            return f"{CMD_FORWARD if self.drive > 0 else CMD_BACKWARD} {round(self.speed)}"
        # Curve: the wheel on the pressed side is the slower, inner one
        outer, inner = self.duty_cycle, self.arc_inner_duty
        left, right = (inner, outer) if self.turn > 0 else (outer, inner)
        return f"{CMD_WHEELS} {left * self.drive} {right * self.drive}"


def main(args=None):
    if not sys.stdin.isatty():
        print("keyboard node needs an interactive terminal (use `ros2 run`, not a launch file)",
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
        node = KeyboardTeleop(key_events)
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
            # then motor_driver's timeout stops the robot instead.
            if rclpy.ok():
                node.on_key(KEY_SPACE, PRESS)
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

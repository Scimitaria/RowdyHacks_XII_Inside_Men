"""Explore by driving to the nearest unmapped space, publishing commands on 'motor_cmd'."""

import math
from collections import deque

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from robot_package.motor_node import CMD_FORWARD, CMD_LEFT, CMD_RIGHT, CMD_STOP


def wrap_angle(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class Explorer(Node):
    """Explore: drive to the nearest edge of the space RTAB-Map has mapped.

    Reads the 2D occupancy grid RTAB-Map publishes on /map (-1 unknown, 0 free,
    100 occupied) and the robot pose from TF (map -> base_link). A frontier is
    a free cell next to unknown ones. A breadth-first search over free cells
    finds the nearest frontier the robot can actually drive to, so ones around
    corners and through doorways count, not just ones in line of sight. The
    depth image is a safety stop, since the grid only updates when RTAB-Map
    adds a node (~1 Hz).
    """

    def __init__(self):
        super().__init__('explorer')

        self.declare_parameter('duty_cycle', 75)         # forward speed
        self.declare_parameter('turn_duty', 50)          # turning speed (slower so the map keeps up)
        self.declare_parameter('obstacle_m', 1.0)        # depth safety stop distance
        self.declare_parameter('robot_radius_m', 0.2)    # keep paths this far from walls
        self.declare_parameter('blind_radius_m', 0.5)    # camera can't see this close: treat as free
        self.declare_parameter('lookahead_m', 0.5)       # steer at the path point this far ahead
        self.declare_parameter('heading_tolerance_deg', 15.0)
        self.duty_cycle = self.get_parameter('duty_cycle').value
        self.turn_duty = self.get_parameter('turn_duty').value
        self.obstacle_m = self.get_parameter('obstacle_m').value
        self.robot_radius_m = self.get_parameter('robot_radius_m').value
        self.blind_radius_m = self.get_parameter('blind_radius_m').value
        self.lookahead_m = self.get_parameter('lookahead_m').value
        self.heading_tolerance = math.radians(self.get_parameter('heading_tolerance_deg').value)

        self.publisher_ = self.create_publisher(String, 'motor_cmd', 10)
        self.create_subscription(Image, 'camera/depth/image_raw', self.on_depth, 10)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd = None
        self.nearest_mm = None
        self.grid = None              # (cells as int8 array [row, col], map info)
        self.replan = False           # set when there's a new map (or obstacle) to plan with
        self.path = None              # [(x, y), ...] in the map frame, ending at a frontier
        self.reached = None           # frontier we just drove up to
        self.dead_ends = []           # frontiers that stayed unknown after we got there
        self.obstacles = []           # (x, y, until): depth camera saw something the map hasn't
        self.status = "waiting for map"

        self.create_timer(0.2, self.control)
        self.create_timer(1.0, self.log)

        self.get_logger().info(
            "\n============================================\n"
            "Explorer started (publishing /motor_cmd)\n"
            "============================================\n"
        )

    def on_depth(self, msg):
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        # Only look at the middle of the frame so the floor and edges don't count
        h, w = depth.shape
        roi = depth[h // 3 : 2 * h // 3, w // 4 : 3 * w // 4]
        valid = roi[roi > 0]  # 0 = no depth reading
        # 1st percentile instead of min() so one noisy pixel doesn't set the distance
        self.nearest_mm = float(np.percentile(valid, 1)) if valid.size else None

    def on_map(self, msg):
        cells = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.grid = (cells, msg.info)
        self.replan = True

    def robot_pose(self):
        """(x, y, yaw) of base_link in the map frame, or None if TF isn't ready."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def plan(self, x, y, min_unknown=6):
        """Path from (x, y) to the nearest reachable frontier, or None if there is none."""
        cells, info = self.grid
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        h, w = cells.shape
        col, row = int((x - ox) / res), int((y - oy) / res)
        if not (0 <= col < w and 0 <= row < h):
            return None
        rows, cols = np.mgrid[0:h, 0:w]
        wx, wy = ox + (cols + 0.5) * res, oy + (rows + 0.5) * res

        def near(px, py, r):
            return np.hypot(wx - px, wy - py) < r

        now = self.get_clock().now()
        self.obstacles = [o for o in self.obstacles if o[2] > now]
        occupied = cells >= 50
        for bx, by, _ in self.obstacles:
            occupied |= near(bx, by, 0.3)
        # Grow walls by the robot's radius so paths keep the whole robot clear
        r = max(1, round(self.robot_radius_m / res))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        blocked = cv2.dilate(occupied.astype(np.uint8), kernel) > 0
        unknown = cells == -1
        # Cells right around the robot are often unknown (camera min range / FOV)
        blind = near(x, y, self.blind_radius_m)
        passable = (~blocked & ~unknown) | (blind & ~occupied)

        # Unknown cells in each 5x5 block (off the map counts as unknown), so a
        # frontier is the edge of real unmapped space, not a 1-cell hole
        padded = np.pad(unknown.astype(np.float32), 2, constant_values=1)
        unknown_nearby = cv2.boxFilter(padded, -1, (5, 5), normalize=False)[2:-2, 2:-2]
        frontier = passable & ~unknown & ~blind & (unknown_nearby >= min_unknown)
        for dx, dy in self.dead_ends:
            frontier &= ~near(dx, dy, 0.5)

        # Breadth-first search from the robot: the first frontier found is the closest by path
        passable, frontier = passable.ravel().tolist(), frontier.ravel().tolist()
        start = row * w + col
        parent = {start: None}
        queue = deque([start])
        while queue:
            i = queue.popleft()
            if frontier[i]:
                break
            r, c = divmod(i, w)
            for j, inside in ((i - w, r > 0), (i + w, r < h - 1), (i - 1, c > 0), (i + 1, c < w - 1)):
                if inside and passable[j] and j not in parent:
                    parent[j] = i
                    queue.append(j)
        else:
            return None

        path = []
        while i is not None:
            r, c = divmod(i, w)
            path.append((ox + (c + 0.5) * res, oy + (r + 0.5) * res))
            i = parent[i]
        path.reverse()
        return path[1::2] + path[-1:]  # every other cell is plenty to steer by

    def waypoint(self, x, y):
        """Next path point at least lookahead_m away, dropping ones we've passed."""
        while self.path and math.dist(self.path[0], (x, y)) < self.lookahead_m:
            self.path.pop(0)
        return self.path[0] if self.path else None

    def control(self):
        pose = self.robot_pose()
        if self.grid is None or pose is None:
            self.status = "waiting for map" if self.grid is None else "waiting for map -> base_link TF"
            self.send(CMD_STOP)
            return
        x, y, yaw = pose
        blocked = self.nearest_mm is not None and self.nearest_mm / 1000 < self.obstacle_m

        if self.replan:
            self.replan = False
            self.path = self.plan(x, y)
            if self.path and self.reached and math.dist(self.path[-1], self.reached) < 0.5:
                # We drove up to this frontier and it's still unknown (glass, black
                # surfaces, beyond depth range): stop going back to it
                self.dead_ends.append(self.reached)
                self.path = self.plan(x, y)
            self.reached = None
            if self.path is None:
                self.status = "no reachable unmapped space left"
        if self.path is None:
            self.send(CMD_STOP)
            return

        goal = self.path[-1]
        target = self.waypoint(x, y)
        if target is None:
            # Close enough to see it: wait for the map to update before picking the next one
            self.status = "reached frontier, waiting for map update"
            self.reached, self.path = goal, None
            self.send(CMD_STOP)
            return

        error = wrap_angle(math.atan2(target[1] - y, target[0] - x) - yaw)
        if abs(error) > self.heading_tolerance:
            self.status = f"turning to frontier ({math.degrees(error):+.0f} deg)"
            self.send(CMD_LEFT if error > 0 else CMD_RIGHT)  # left = counter-clockwise = +yaw
        elif blocked:
            # The map hasn't caught up with what the camera sees: plan around it for a while
            self.status = "obstacle ahead, planning around it"
            d = self.nearest_mm / 1000
            until = self.get_clock().now() + Duration(seconds=10)
            self.obstacles.append((x + d * math.cos(yaw), y + d * math.sin(yaw), until))
            self.replan, self.path = True, None
            self.send(CMD_STOP)
        else:
            dist = math.dist(goal, (x, y))
            self.status = f"driving to frontier ({dist:.1f} m away)"
            self.send(CMD_FORWARD)

    def log(self):
        nearest = "no depth data" if self.nearest_mm is None else f"{self.nearest_mm / 1000:.2f} m"
        self.get_logger().info(f"{self.status} | nearest object: {nearest}")

    def send(self, cmd):
        # Publish every tick: motor_driver stops the robot if commands stop arriving
        duty = self.duty_cycle if cmd == CMD_FORWARD else self.turn_duty
        line = f"{cmd} {duty}"
        self.publisher_.publish(String(data=line))
        if cmd != self.cmd:
            self.cmd = cmd
            self.get_logger().info(f"motor_cmd: {line}")


def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Stop on exit. After Ctrl+C rclpy is already shut down and can't publish;
        # then motor_driver's timeout stops the robot instead.
        if rclpy.ok():
            node.send(CMD_STOP)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

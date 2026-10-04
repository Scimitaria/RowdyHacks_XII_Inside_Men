"""Explore by sending Nav2 goals at the nearest unmapped space on RTAB-Map's /map."""

import math
from collections import deque

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


class Goal:
    """A frontier sent to Nav2, and the goal handle once Nav2 accepts it."""

    def __init__(self, xy, map_count):
        self.xy = xy
        self.map_count = map_count    # maps received when it was sent
        self.handle = None
        self.canceling = False


class Explorer(Node):
    """Explore: send Nav2 to the nearest edge of the space RTAB-Map has mapped.

    Reads the 2D occupancy grid RTAB-Map publishes on /map (-1 unknown, 0 free,
    100 occupied) and the robot pose from TF (map -> base_link). A frontier is
    a free cell next to unknown ones. A breadth-first search over free cells
    finds the nearest frontier the robot can actually drive to, so ones around
    corners and through doorways count, not just ones in line of sight.
    Frontiers closer than min_frontier_dist_m by path are passed over, so the
    robot makes fewer, longer trips instead of stopping every few steps; if
    only close ones are left, the nearest is used. That frontier goes to Nav2 as a NavigateToPose goal; Nav2 plans, drives and
    avoids obstacles.

    If the camera sees past the frontier before the robot gets there, the next
    one is sent in its place; Nav2 switches goals without stopping the robot. Frontiers Nav2 can't reach, or that
    stay unknown after the robot arrives (glass, black surfaces, beyond depth
    range), are skipped from then on.

    keyboard_odom can take the robot over: True on /manual_control cancels the
    current goal and pauses exploring, False resumes it.
    """

    def __init__(self):
        super().__init__('explorer')

        self.declare_parameter('robot_radius_m', 0.2)    # keep goals this far from walls
        self.declare_parameter('blind_radius_m', 0.5)    # camera can't see this close: treat as free
        self.declare_parameter('min_unknown', 6)         # unknown cells in a 5x5 block to count as a frontier
        self.declare_parameter('dead_end_radius_m', 0.5)
        self.declare_parameter('min_frontier_dist_m', 1.0)  # prefer frontiers at least this far by path
        self.declare_parameter('initial_spin', True)     # look all around before the first goal
        self.robot_radius_m = self.get_parameter('robot_radius_m').value
        self.blind_radius_m = self.get_parameter('blind_radius_m').value
        self.min_unknown = self.get_parameter('min_unknown').value
        self.dead_end_radius_m = self.get_parameter('dead_end_radius_m').value
        self.min_frontier_dist_m = self.get_parameter('min_frontier_dist_m').value

        # RTAB-Map publishes /map latched
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, map_qos)
        # keyboard_odom publishes it latched too
        self.create_subscription(Bool, 'manual_control', self.on_manual_control, map_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin = ActionClient(self, Spin, 'spin')

        self.grid = None              # (cells as int8 array [row, col], map info)
        self.map_count = 0            # maps received so far
        self.goal = None              # Goal being driven to
        self.reached = None           # (frontier, map_count): arrived, waiting to see if it got mapped
        self.dead_ends = []           # frontiers to skip
        self.spin_state = 'pending' if self.get_parameter('initial_spin').value else 'done'
        self.spin_handle = None
        self.manual = False           # keyboard_odom is driving: send no goals
        self.status = None

        self.create_timer(1.0, self.control)

        self.get_logger().info(
            "\n============================================\n"
            "Explorer started (sending Nav2 goals)\n"
            "============================================\n"
        )

    def on_map(self, msg):
        cells = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.grid = (cells, msg.info)
        self.map_count += 1

    def on_manual_control(self, msg):
        if msg.data == self.manual:
            return
        self.manual = msg.data
        if not self.manual:
            self.get_logger().info("Keyboard handed control back, resuming exploration")
            return
        self.get_logger().info("Keyboard took control, cancelling the current goal")
        if self.goal is not None and self.goal.handle is not None:
            self.goal.handle.cancel_goal_async()
        # Clearing these makes the goal callbacks ignore the cancelled goal
        self.goal = None
        self.reached = None
        if self.spin_state == 'spinning' and self.spin_handle is not None:
            self.spin_handle.cancel_goal_async()
        self.spin_state = 'done'

    def robot_pose(self):
        """(x, y) of base_link in the map frame, or None if TF isn't ready."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def unknown_nearby(self, unknown):
        """Unknown cells in each 5x5 block (off the map counts as unknown)."""
        padded = np.pad(unknown.astype(np.float32), 2, constant_values=1)
        return cv2.boxFilter(padded, -1, (5, 5), normalize=False)[2:-2, 2:-2]

    def is_frontier(self, xy):
        """Whether the map still shows unmapped space next to xy."""
        cells, info = self.grid
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        h, w = cells.shape
        col, row = int((xy[0] - ox) / res), int((xy[1] - oy) / res)
        if not (0 <= col < w and 0 <= row < h):
            return True
        block = np.pad(cells, 2, constant_values=-1)[row:row + 5, col:col + 5]
        return cells[row, col] < 50 and np.count_nonzero(block == -1) >= self.min_unknown

    def plan(self, x, y):
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

        occupied = cells >= 50
        # Grow walls by the robot's radius so goals keep the whole robot clear
        r = max(1, round(self.robot_radius_m / res))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        blocked = cv2.dilate(occupied.astype(np.uint8), kernel) > 0
        unknown = cells == -1
        # Cells right around the robot are often unknown (camera min range / FOV)
        blind = near(x, y, self.blind_radius_m)
        passable = (~blocked & ~unknown) | (blind & ~occupied)

        # A frontier is the edge of real unmapped space, not a 1-cell hole
        frontier = (passable & ~unknown & ~blind
                    & (self.unknown_nearby(unknown) >= self.min_unknown))
        for dx, dy in self.dead_ends:
            frontier &= ~near(dx, dy, self.dead_end_radius_m)

        # Breadth-first search from the robot: frontiers come out closest by path first.
        # Take the first one at least min_frontier_dist_m away.
        passable, frontier = passable.ravel().tolist(), frontier.ravel().tolist()
        min_steps = self.min_frontier_dist_m / res
        start = row * w + col
        parent = {start: None}
        steps = {start: 0}
        nearest = None
        queue = deque([start])
        while queue:
            i = queue.popleft()
            if frontier[i]:
                if steps[i] >= min_steps:
                    break
                if nearest is None:
                    nearest = i
            r, c = divmod(i, w)
            for j, inside in ((i - w, r > 0), (i + w, r < h - 1), (i - 1, c > 0), (i + 1, c < w - 1)):
                if inside and passable[j] and j not in parent:
                    parent[j] = i
                    steps[j] = steps[i] + 1
                    queue.append(j)
        else:
            # Only close frontiers left: go to the nearest rather than stop exploring
            if nearest is None:
                return None
            i = nearest

        path = []
        while i is not None:
            r, c = divmod(i, w)
            path.append((ox + (c + 0.5) * res, oy + (r + 0.5) * res))
            i = parent[i]
        path.reverse()
        return path

    def control(self):
        if self.manual:
            return self.set_status("paused: keyboard_odom in control")
        if self.grid is None:
            return self.set_status("waiting for /map")
        if not self.nav.server_is_ready():
            return self.set_status("waiting for Nav2 (navigate_to_pose)")
        if self.spin_state != 'done':
            if self.spin_state == 'pending':
                self.start_spin()
            return
        pose = self.robot_pose()
        if pose is None:
            return self.set_status("waiting for map -> base_link TF")

        goal = self.goal
        if goal is not None:
            # Only judge it on a map made after it was sent
            if (goal.handle and not goal.canceling and self.map_count > goal.map_count
                    and not self.is_frontier(goal.xy)):
                path = self.plan(*pose)
                if path is None:
                    self.get_logger().info("Frontier mapped before getting there, none left")
                    goal.canceling = True
                    goal.handle.cancel_goal_async()
                else:
                    # Nav2 aborts the old goal for the new one; its result is ignored
                    self.get_logger().info("Frontier mapped before getting there, switching to the next one")
                    self.send_goal(path)
            else:
                dist = math.dist(goal.xy, pose)
                self.set_status(f"navigating to frontier ({goal.xy[0]:.1f}, {goal.xy[1]:.1f}), "
                                f"{dist:.1f} m away")
            return

        if self.reached is not None:
            xy, count = self.reached
            # Give RTAB-Map a couple of updates to add what the camera sees from here
            if self.map_count < count + 2:
                return self.set_status("reached frontier, waiting for map update")
            if self.is_frontier(xy):
                self.skip(xy, "still unmapped after getting there")
            self.reached = None

        path = self.plan(*pose)
        if path is None:
            blacklisted = f" ({len(self.dead_ends)} skipped)" if self.dead_ends else ""
            return self.set_status(f"exploration done: no reachable unmapped space left{blacklisted}")
        self.send_goal(path)

    def start_spin(self):
        if not self.spin.server_is_ready():
            return self.set_status("waiting for Nav2 (spin)")
        self.spin_state = 'spinning'
        self.set_status("spinning to look around")
        goal = Spin.Goal(target_yaw=2 * math.pi)
        goal.time_allowance = Duration(seconds=30).to_msg()

        def on_result(future):
            self.spin_state = 'done'

        def on_response(future):
            handle = future.result()
            if not handle.accepted:
                self.spin_state = 'done'
                return
            if self.manual:
                # Keyboard took over while the goal was on its way
                handle.cancel_goal_async()
                return
            self.spin_handle = handle
            handle.get_result_async().add_done_callback(on_result)

        self.spin.send_goal_async(goal).add_done_callback(on_response)

    def send_goal(self, path):
        xy = path[-1]
        # Face the way the path arrives, i.e. toward the unknown space past the frontier
        before = path[max(0, len(path) - 6)]
        yaw = math.atan2(xy[1] - before[1], xy[0] - before[0])

        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = 'map'
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = xy
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)

        goal = self.goal = Goal(xy, self.map_count)
        self.get_logger().info(f"New frontier goal ({xy[0]:.2f}, {xy[1]:.2f}), "
                               f"{len(path) * self.grid[1].resolution:.1f} m by path")

        def on_result(future):
            if self.goal is not goal:
                return
            self.goal = None
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.reached = (xy, self.map_count)
            elif status != GoalStatus.STATUS_CANCELED:
                self.skip(xy, f"Nav2 couldn't get there (status {status})")

        def on_response(future):
            if self.goal is not goal:
                handle = future.result()
                if self.manual and handle.accepted:
                    # Keyboard took over while the goal was on its way
                    handle.cancel_goal_async()
                return
            handle = future.result()
            if not handle.accepted:
                self.goal = None
                self.skip(xy, "Nav2 rejected the goal")
                return
            goal.handle = handle
            handle.get_result_async().add_done_callback(on_result)

        self.nav.send_goal_async(msg).add_done_callback(on_response)

    def skip(self, xy, why):
        self.dead_ends.append(xy)
        self.get_logger().warn(f"Skipping frontier ({xy[0]:.2f}, {xy[1]:.2f}): {why}")

    def set_status(self, status):
        # Log on change, and every 5 s while it stays the same
        if status != self.status:
            self.status = status
            self.get_logger().info(status)
        else:
            self.get_logger().info(status, throttle_duration_sec=5.0)

    def cancel(self):
        """Cancel the current goal and wait briefly for Nav2 to acknowledge."""
        if self.goal is None or self.goal.handle is None:
            return
        future = self.goal.handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        self.get_logger().info("Cancelled the current goal")


def main(args=None):
    # Handle Ctrl+C ourselves so rclpy is still up to cancel the Nav2 goal
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cancel()
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

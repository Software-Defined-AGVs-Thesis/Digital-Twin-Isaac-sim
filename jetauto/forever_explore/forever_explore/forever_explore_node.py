"""forever_explore_node

Sits next to explore_lite and turns it into an infinite explorer.

Flow:
  1. explore_lite drives smart frontier-based exploration (default behavior).
  2. When explore_lite has no more frontiers it publishes
     EXPLORATION_COMPLETE on /explore/status.
  3. We catch that, log a loud banner, clear Nav2's global costmap (so
     stale obstacle blobs don't pen the robot in), pick a random reachable
     point from the SLAM occupancy grid, and send it as a NavigateToPose
     goal. Once the random goal finishes (or times out), we publish True
     to /explore/resume so explore_lite re-runs frontier search on the
     freshly-cleared costmap.
  4. Repeat forever.

Every state transition is logged at INFO level with a banner so it
shows up cleanly in the tmux logs.
"""

import math
import random
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap

try:
    from explore_lite_msgs.msg import ExploreStatus
    EXPLORE_MSG_AVAILABLE = True
except ImportError:  # explore_lite not built — node still useful as random walker
    ExploreStatus = None
    EXPLORE_MSG_AVAILABLE = False


BANNER = "=" * 72


class ForeverExplore(Node):
    def __init__(self):
        super().__init__('forever_explore')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('clear_global_service',
                               '/global_costmap/clear_entirely_global_costmap')
        self.declare_parameter('clear_local_service',
                               '/local_costmap/clear_entirely_local_costmap')
        self.declare_parameter('nav_action', '/navigate_to_pose')
        self.declare_parameter('explore_resume_topic', '/explore/resume')
        self.declare_parameter('explore_status_topic', '/explore/status')
        self.declare_parameter('random_goal_timeout_sec', 60.0)
        self.declare_parameter('cooldown_after_goal_sec', 5.0)
        self.declare_parameter('min_dist_from_robot', 1.0)
        self.declare_parameter('max_random_attempts', 200)

        self.map_topic = self.get_parameter('map_topic').value
        self.clear_global = self.get_parameter('clear_global_service').value
        self.clear_local = self.get_parameter('clear_local_service').value
        self.nav_action = self.get_parameter('nav_action').value
        self.resume_topic = self.get_parameter('explore_resume_topic').value
        self.status_topic = self.get_parameter('explore_status_topic').value
        self.goal_timeout = float(self.get_parameter('random_goal_timeout_sec').value)
        self.cooldown = float(self.get_parameter('cooldown_after_goal_sec').value)
        self.min_dist = float(self.get_parameter('min_dist_from_robot').value)
        self.max_attempts = int(self.get_parameter('max_random_attempts').value)

        self.latest_map = None
        self.cycle = 0
        self.busy = False
        self.lock = threading.Lock()

        map_qos = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, self.map_topic,
                                 self._on_map, map_qos)

        if EXPLORE_MSG_AVAILABLE:
            self.create_subscription(ExploreStatus, self.status_topic,
                                     self._on_status, 10)
        else:
            self.get_logger().warn(
                "explore_lite_msgs not found — running in random-only "
                "fallback mode (no smart frontier exploration).")

        self.resume_pub = self.create_publisher(Bool, self.resume_topic, 10)
        # NOTE: clear-costmap clients are created PER-CYCLE in _forever_cycle.
        # A persistent client created at startup gets a stale internal handle
        # if Nav2's costmap services come up later — wait_for_service() then
        # times out forever even though the service is reachable to fresh
        # clients. Recreating each cycle costs us nothing and avoids that.
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action)

        # Random-only fallback: if explore_lite never publishes, we still
        # want to keep moving. Use monotonic wall time, NOT sim time, because
        # use_sim_time=true makes get_clock().now() jump from 0 to the sim
        # epoch (~1.7e9s) once /clock starts publishing — which would make
        # any elapsed-since-init calc wildly wrong.
        self.startup_monotonic = time.monotonic()
        self.last_status_monotonic = None  # set on first /explore/status
        self.fallback_grace_sec = 90.0     # wait this long before first fallback
        self.fallback_silence_sec = 120.0  # then trigger if silent this long
        self.fallback_timer = self.create_timer(15.0, self._fallback_tick)

        self.get_logger().info(BANNER)
        self.get_logger().info("forever_explore READY")
        self.get_logger().info(
            f"  smart mode  : explore_lite (frontier-based) "
            f"{'ON' if EXPLORE_MSG_AVAILABLE else 'OFF (msgs missing)'}")
        self.get_logger().info(
            f"  forever mode: random reachable goals after each completion")
        self.get_logger().info(BANNER)

    # ---- callbacks ----------------------------------------------------------

    def _on_map(self, msg: OccupancyGrid):
        self.latest_map = msg

    def _on_status(self, msg):
        self.last_status_monotonic = time.monotonic()
        status = msg.status
        if status == ExploreStatus.EXPLORATION_STARTED:
            self.get_logger().info("[explore_lite] EXPLORATION_STARTED — frontier search running.")
        elif status == ExploreStatus.EXPLORATION_IN_PROGRESS:
            self.get_logger().info("[explore_lite] EXPLORATION_IN_PROGRESS — resumed.")
        elif status == ExploreStatus.EXPLORATION_PAUSED:
            self.get_logger().info("[explore_lite] EXPLORATION_PAUSED.")
        elif status == ExploreStatus.EXPLORATION_COMPLETE:
            self.get_logger().info("[explore_lite] EXPLORATION_COMPLETE — no frontiers left.")
            self._enter_forever_mode("exploration_complete")
        elif status == ExploreStatus.RETURNING_TO_ORIGIN:
            self.get_logger().info("[explore_lite] RETURNING_TO_ORIGIN.")
        elif status == ExploreStatus.RETURNED_TO_ORIGIN:
            self.get_logger().info("[explore_lite] RETURNED_TO_ORIGIN — kicking forever mode.")
            self._enter_forever_mode("returned_to_origin")

    def _fallback_tick(self):
        # If we haven't heard anything from explore_lite for a long time AND
        # we're not currently driving a random goal, fire one anyway.
        if self.busy:
            return
        # Always grant the stack a startup grace period. Discovery + Nav2
        # bringup can easily take 30-60s after this node spins up.
        since_start = time.monotonic() - self.startup_monotonic
        if since_start < self.fallback_grace_sec:
            return
        if self.last_status_monotonic is None:
            self.get_logger().warn(
                f"No /explore/status received in first {since_start:.0f}s — "
                f"explore_lite may not be running. Falling back to random goals.")
            self._enter_forever_mode("status_never_seen")
            return
        elapsed = time.monotonic() - self.last_status_monotonic
        if elapsed > self.fallback_silence_sec:
            self.get_logger().warn(
                f"No /explore/status for {elapsed:.0f}s — random fallback to keep moving.")
            self._enter_forever_mode("status_silence")

    # ---- forever-mode logic -------------------------------------------------

    def _enter_forever_mode(self, reason: str):
        with self.lock:
            if self.busy:
                self.get_logger().info(f"  (already in forever cycle, ignoring trigger '{reason}')")
                return
            self.busy = True
        self.cycle += 1
        n = self.cycle

        self.get_logger().info("")
        self.get_logger().info(BANNER)
        self.get_logger().info(f"  FOREVER MODE — cycle #{n}  (trigger: {reason})")
        self.get_logger().info(f"  Room appears fully mapped. Clearing costmaps and")
        self.get_logger().info(f"  dispatching a RANDOM reachable goal so we never stop.")
        self.get_logger().info(BANNER)

        # Run the rest off-thread so we don't block executor callbacks.
        threading.Thread(target=self._forever_cycle, args=(n,), daemon=True).start()

    def _forever_cycle(self, n: int):
        try:
            # Fresh clients each cycle — see note in __init__.
            global_cli = self.create_client(ClearEntireCostmap, self.clear_global)
            local_cli = self.create_client(ClearEntireCostmap, self.clear_local)
            try:
                self._call_clear(global_cli, "global costmap")
                self._call_clear(local_cli, "local costmap")
            finally:
                self.destroy_client(global_cli)
                self.destroy_client(local_cli)

            goal = self._pick_random_goal()
            if goal is None:
                self.get_logger().error(
                    f"[cycle #{n}] could not sample a reachable random goal — "
                    f"will retry on next trigger.")
                self._resume_explore()
                return

            x = goal.pose.position.x
            y = goal.pose.position.y
            self.get_logger().info(
                f"[cycle #{n}] RANDOM GOAL → x={x:+.2f} y={y:+.2f} (sending to Nav2)")

            ok = self._send_nav_goal(goal, n)
            if ok:
                self.get_logger().info(f"[cycle #{n}] random goal finished cleanly.")
            else:
                self.get_logger().warn(f"[cycle #{n}] random goal aborted/timed out.")

            # Cooldown so Nav2 settles, then nudge explore_lite to re-search.
            time.sleep(self.cooldown)
            self._resume_explore()
            self.get_logger().info(f"[cycle #{n}] resumed explore_lite — back to smart mode.")
        finally:
            self.busy = False

    def _call_clear(self, client, label: str):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"  clear-{label} service unavailable after 5s, skipping.")
            return
        future = client.call_async(ClearEntireCostmap.Request())
        if self._wait_future(future, timeout=5.0):
            self.get_logger().info(f"  cleared {label}.")
        else:
            self.get_logger().warn(f"  clear-{label} timed out.")

    def _pick_random_goal(self):
        m = self.latest_map
        if m is None:
            self.get_logger().warn("  no /map yet — cannot sample goal.")
            return None

        w, h = m.info.width, m.info.height
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        data = m.data

        free = [i for i, v in enumerate(data) if v == 0]
        if not free:
            self.get_logger().warn("  /map has no free cells.")
            return None

        for _ in range(self.max_attempts):
            idx = random.choice(free)
            mx = idx % w
            my = idx // w
            x = ox + (mx + 0.5) * res
            y = oy + (my + 0.5) * res

            # Avoid sampling on top of an obstacle or right next to the robot.
            if not self._has_clearance(data, w, h, mx, my, cells=3):
                continue

            goal = PoseStamped()
            goal.header.frame_id = m.header.frame_id or 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = x
            goal.pose.position.y = y
            yaw = random.uniform(-math.pi, math.pi)
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.orientation.w = math.cos(yaw / 2.0)
            return goal
        return None

    @staticmethod
    def _has_clearance(data, w, h, mx, my, cells=3):
        for dy in range(-cells, cells + 1):
            for dx in range(-cells, cells + 1):
                nx, ny = mx + dx, my + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    return False
                v = data[ny * w + nx]
                if v != 0:  # unknown (-1) or occupied (>0) too close
                    return False
        return True

    def _send_nav_goal(self, pose: PoseStamped, n: int) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(f"[cycle #{n}] navigate_to_pose action server unavailable after 30s.")
            return False
        msg = NavigateToPose.Goal()
        msg.pose = pose
        send_future = self.nav_client.send_goal_async(msg)
        if not self._wait_future(send_future, timeout=10.0):
            self.get_logger().error(f"[cycle #{n}] Nav2 didn't accept goal in time.")
            return False
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f"[cycle #{n}] random goal rejected by Nav2.")
            return False
        result_future = gh.get_result_async()
        return self._wait_future(result_future, timeout=self.goal_timeout)

    @staticmethod
    def _wait_future(future, timeout: float) -> bool:
        # Safe to call from a worker thread because the main thread runs
        # rclpy.spin(node), which services callbacks (including the ones
        # that resolve this future).
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)
        return True

    def _resume_explore(self):
        if not EXPLORE_MSG_AVAILABLE:
            return
        msg = Bool()
        msg.data = True
        self.resume_pub.publish(msg)


def main():
    rclpy.init()
    node = ForeverExplore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

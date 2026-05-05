#!/usr/bin/env python3
"""StuckGuard — detect "wheels spinning but robot isn't actually moving" and
react identically to CliffGuard (zero Twist on /cmd_vel + /cmd_vel_nav,
cancel /navigate_to_pose, PAUSE Nav2 via lifecycle_manager_navigation).

Detection signals over a rolling window (default 7s):
  1. /cmd_vel   — Nav2 commanding motion        ("we're trying to drive")
  2. /odom      — wheel odom reports motion     ("wheels are spinning")
  3. TF map -> base_footprint — SLAM pose       ("did we actually move?")

Trip when all three hold for the full window:
  - cmd_vel above min thresholds the entire window, AND
  - odom linear speed above min threshold the entire window, AND
  - SLAM pose displacement over the window < epsilon.

The /odom + TF discrepancy is the key signal: encoders happily report motion
in sand while SLAM scan-matching against LIDAR shows no real progress.
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, Float32
from std_srvs.srv import Trigger
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from action_msgs.srv import CancelGoal
from unique_identifier_msgs.msg import UUID
from nav2_msgs.srv import ManageLifecycleNodes

import tf2_ros
from rclpy.time import Time
from rclpy.duration import Duration

NAV2_LIFECYCLE_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'smoother_server',
    'velocity_smoother',
]


class StuckGuard(Node):
    def __init__(self):
        super().__init__('stuck_guard')

        # --- topics / frames ---
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        # velocity_smoother subscribes to /cmd_vel_nav and re-publishes to
        # /cmd_vel — same trick cliff_guard uses, spam BOTH on stop.
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('nav_action', '/navigate_to_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        # --- detection thresholds ---
        self.declare_parameter('window_sec', 7.0)
        self.declare_parameter('cmd_vel_min_linear', 0.05)   # m/s
        self.declare_parameter('cmd_vel_min_angular', 0.1)   # rad/s
        self.declare_parameter('wheel_odom_min_linear', 0.03)  # m/s — wheels are spinning
        self.declare_parameter('pose_displacement_max', 0.05)  # m over window — we didn't move

        # --- nav2 management (mirrors cliff_guard) ---
        self.declare_parameter('nav2_nodes', NAV2_LIFECYCLE_NODES)
        self.declare_parameter('manage_nav2', True)
        self.declare_parameter('lifecycle_manager_service',
                               '/lifecycle_manager_navigation/manage_nodes')
        self.declare_parameter('lifecycle_call_timeout', 3.0)
        self.declare_parameter('resume_max_data_age', 1.5)  # s; reject resume if odom/TF stale

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_vel_nav_topic = self.get_parameter('cmd_vel_nav_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.nav_action = self.get_parameter('nav_action').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.WINDOW_SEC = float(self.get_parameter('window_sec').value)
        self.CMD_MIN_LIN = float(self.get_parameter('cmd_vel_min_linear').value)
        self.CMD_MIN_ANG = float(self.get_parameter('cmd_vel_min_angular').value)
        self.WHEEL_MIN_LIN = float(self.get_parameter('wheel_odom_min_linear').value)
        self.POSE_DISP_MAX = float(self.get_parameter('pose_displacement_max').value)

        self.nav2_nodes = list(self.get_parameter('nav2_nodes').value)
        self.manage_nav2 = bool(self.get_parameter('manage_nav2').value)
        self.lifecycle_manager_service = self.get_parameter('lifecycle_manager_service').value
        self.lifecycle_timeout = float(self.get_parameter('lifecycle_call_timeout').value)
        self.resume_max_data_age = float(self.get_parameter('resume_max_data_age').value)

        # --- state ---
        self.emergency_stop = False
        self.vr_override_active = False
        self.currently_stuck = False

        # rolling windows: deque of (t_ns, value)
        self.cmd_window = deque()
        self.odom_window = deque()
        self.pose_window = deque()  # (t_ns, x, y)

        self.last_cmd_recv_ns = 0
        self.last_odom_recv_ns = 0
        self.last_pose_recv_ns = 0
        self.last_pose_disp = float('nan')
        self.frames_cmd = 0
        self.frames_odom = 0
        self.frames_pose = 0

        # --- ROS plumbing ---
        self.cb_sensors = MutuallyExclusiveCallbackGroup()
        self.cb_services = ReentrantCallbackGroup()

        sensor_qos = QoSProfile(depth=20)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.sub_cmd = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_cb, 10,
            callback_group=self.cb_sensors)
        self.sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self.odom_cb, sensor_qos,
            callback_group=self.cb_sensors)
        self.sub_resume = self.create_subscription(
            Bool, '/stuck_guard/resume', self.resume_topic_cb, 10,
            callback_group=self.cb_services)
        self.sub_vr_active = self.create_subscription(
            Bool, '/vr_override/active', self.vr_active_cb, 10,
            callback_group=self.cb_services)

        self.pub_stop = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_stop_nav = self.create_publisher(Twist, self.cmd_vel_nav_topic, 10)
        self.pub_alert = self.create_publisher(String, '/stuck_guard/alert', latched_qos)
        self.pub_status = self.create_publisher(Bool, '/stuck_guard/status', latched_qos)
        self.pub_disp = self.create_publisher(Float32, '/stuck_guard/displacement', 10)

        # See cliff_guard for explanation of cancel-all goal_id convention.
        self.cancel_nav_client = self.create_client(
            CancelGoal, f'{self.nav_action}/_action/cancel_goal',
            callback_group=self.cb_services)

        self.srv_resume = self.create_service(
            Trigger, '/stuck_guard/resume_nav', self.resume_service_cb,
            callback_group=self.cb_services)

        self.lifecycle_mgr_client = self.create_client(
            ManageLifecycleNodes, self.lifecycle_manager_service,
            callback_group=self.cb_services)

        self.change_state_clients = {}
        self.get_state_clients = {}
        if self.manage_nav2:
            for name in self.nav2_nodes:
                self.change_state_clients[name] = self.create_client(
                    ChangeState, f'/{name}/change_state',
                    callback_group=self.cb_services)
                self.get_state_clients[name] = self.create_client(
                    GetState, f'/{name}/get_state',
                    callback_group=self.cb_services)

        # TF buffer for SLAM-corrected pose (map -> base_footprint).
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 10 Hz pose sampler — pulling TF in a timer keeps the detection loop
        # decoupled from /cmd_vel and /odom callback rates.
        self.pose_timer = self.create_timer(0.1, self.pose_sample_cb,
                                            callback_group=self.cb_sensors)
        # 10 Hz detection loop — evaluates the rolling window.
        self.detect_timer = self.create_timer(0.1, self.detect_cb,
                                              callback_group=self.cb_sensors)
        self.telemetry_timer = self.create_timer(0.5, self.telemetry_cb,
                                                 callback_group=self.cb_sensors)

        self._publish_status()
        self._publish_alert('READY')
        self.get_logger().info(
            f'StuckGuard started — cmd_vel={self.cmd_vel_topic}, odom={self.odom_topic}, '
            f'tf={self.map_frame}->{self.base_frame}, window={self.WINDOW_SEC}s, '
            f'manage_nav2={self.manage_nav2}')

    # ---------- subscriptions ----------

    def cmd_cb(self, msg: Twist):
        now = self.get_clock().now().nanoseconds
        self.last_cmd_recv_ns = now
        self.frames_cmd += 1
        lin = abs(msg.linear.x)
        ang = abs(msg.angular.z)
        commanding = (lin >= self.CMD_MIN_LIN) or (ang >= self.CMD_MIN_ANG)
        self.cmd_window.append((now, commanding))
        self._trim(self.cmd_window, now)

    def odom_cb(self, msg: Odometry):
        now = self.get_clock().now().nanoseconds
        self.last_odom_recv_ns = now
        self.frames_odom += 1
        v = msg.twist.twist.linear
        speed = math.sqrt(v.x * v.x + v.y * v.y)
        spinning = speed >= self.WHEEL_MIN_LIN
        self.odom_window.append((now, spinning))
        self._trim(self.odom_window, now)

    def pose_sample_cb(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.05))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        now = self.get_clock().now().nanoseconds
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        self.last_pose_recv_ns = now
        self.frames_pose += 1
        self.pose_window.append((now, x, y))
        self._trim(self.pose_window, now)

    def vr_active_cb(self, msg: Bool):
        self.vr_override_active = msg.data

    def resume_topic_cb(self, msg: Bool):
        if not msg.data:
            return
        ok, reason = self._try_resume()
        if ok:
            self.get_logger().info('Resume via topic accepted.')
        else:
            self.get_logger().warn(f'Resume via topic rejected: {reason}')

    def resume_service_cb(self, request, response):
        ok, reason = self._try_resume()
        response.success = ok
        response.message = reason
        return response

    # ---------- detection ----------

    def _trim(self, dq, now_ns):
        cutoff = now_ns - int(self.WINDOW_SEC * 1e9)
        while dq and dq[0][0] < cutoff:
            dq.popleft()

    def _window_full(self, dq):
        """True if the deque spans at least WINDOW_SEC."""
        if len(dq) < 2:
            return False
        span = (dq[-1][0] - dq[0][0]) / 1e9
        return span >= self.WINDOW_SEC * 0.9  # 10% slack for jitter

    def _all_true(self, dq):
        return all(v for (_, v) in dq)

    def _pose_displacement(self):
        """Max pairwise displacement in the pose window (catches drift-and-return)."""
        if len(self.pose_window) < 2:
            return float('nan')
        xs = [p[1] for p in self.pose_window]
        ys = [p[2] for p in self.pose_window]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        return math.hypot(x_max - x_min, y_max - y_min)

    def detect_cb(self):
        if self.emergency_stop or self.vr_override_active:
            self.currently_stuck = False
            return

        if not (self._window_full(self.cmd_window)
                and self._window_full(self.odom_window)
                and self._window_full(self.pose_window)):
            self.currently_stuck = False
            return

        cmd_active = self._all_true(self.cmd_window)
        wheels_spinning = self._all_true(self.odom_window)
        disp = self._pose_displacement()
        self.last_pose_disp = disp
        pose_static = math.isfinite(disp) and disp < self.POSE_DISP_MAX

        self.currently_stuck = cmd_active and wheels_spinning and pose_static
        if self.currently_stuck:
            self._trigger_emergency_stop(disp)

    def telemetry_cb(self):
        d = Float32()
        d.data = self.last_pose_disp if math.isfinite(self.last_pose_disp) else -1.0
        self.pub_disp.publish(d)
        self.get_logger().info(
            f'cmd_frames={self.frames_cmd} odom_frames={self.frames_odom} '
            f'pose_frames={self.frames_pose} '
            f'win_disp={self.last_pose_disp:.3f}m thr={self.POSE_DISP_MAX:.3f}m '
            f'stuck={self.currently_stuck} estop={self.emergency_stop} '
            f'vr={self.vr_override_active}',
            throttle_duration_sec=1.0)

    # ---------- response (mirrors cliff_guard) ----------

    def _trigger_emergency_stop(self, disp):
        self.emergency_stop = True
        self.get_logger().error(
            f'!!! STUCK DETECTED !!! pose displacement={disp:.3f}m over '
            f'{self.WINDOW_SEC:.1f}s while wheels were spinning and Nav2 was '
            f'commanding motion — STOPPING & PAUSING NAV2')
        zero = Twist()
        self.pub_stop.publish(zero)
        self.pub_stop_nav.publish(zero)
        self._cancel_nav_goals()
        if self.manage_nav2:
            self._pause_nav2()
        self._publish_alert(
            f'STUCK_DETECTED displacement={disp:.3f}m. Robot is halted and Nav2 is paused. '
            f'Free the robot (push, change surface, etc.), then publish '
            f'`true` on /stuck_guard/resume or call /stuck_guard/resume_nav.')
        self._publish_status()

    def _cancel_nav_goals(self):
        if not self.cancel_nav_client.service_is_ready():
            self.get_logger().warn(
                f'{self.nav_action}/_action/cancel_goal not ready — skipping cancel.')
            return
        req = CancelGoal.Request()
        req.goal_info.goal_id = UUID()
        req.goal_info.stamp.sec = 0
        req.goal_info.stamp.nanosec = 0
        self.cancel_nav_client.call_async(req)
        self.get_logger().info(f'Cancel-all dispatched to {self.nav_action}.')

    def _try_resume(self):
        if not self.emergency_stop:
            return True, 'Not in emergency stop; nothing to do.'
        now = self.get_clock().now().nanoseconds
        odom_age = (now - self.last_odom_recv_ns) / 1e9 if self.last_odom_recv_ns else float('inf')
        pose_age = (now - self.last_pose_recv_ns) / 1e9 if self.last_pose_recv_ns else float('inf')
        if odom_age > self.resume_max_data_age or pose_age > self.resume_max_data_age:
            msg = (f'Stale data — odom_age={odom_age:.2f}s, pose_age={pose_age:.2f}s '
                   f'(>{self.resume_max_data_age:.2f}s). Wait for fresh data before resuming.')
            self._publish_alert(f'RESUME_REJECTED {msg}')
            return False, msg
        # Clear the windows so we don't immediately re-trip on stale data.
        self.cmd_window.clear()
        self.odom_window.clear()
        self.pose_window.clear()
        self.get_logger().info('Resume accepted — reactivating Nav2.')
        if self.manage_nav2:
            self._resume_nav2()
        self.emergency_stop = False
        self.currently_stuck = False
        self._publish_status()
        self._publish_alert('RESUMED — Nav2 reactivated, robot operational.')
        return True, 'Nav2 resumed.'

    def _pause_nav2(self):
        self._call_lifecycle_manager(ManageLifecycleNodes.Request.PAUSE,
                                     fallback_transition=Transition.TRANSITION_DEACTIVATE)

    def _resume_nav2(self):
        self._call_lifecycle_manager(ManageLifecycleNodes.Request.RESUME,
                                     fallback_transition=Transition.TRANSITION_ACTIVATE)

    def _call_lifecycle_manager(self, command, fallback_transition):
        if not self.lifecycle_mgr_client.service_is_ready():
            self.get_logger().warn(
                f'{self.lifecycle_manager_service} not ready — '
                f'falling back to per-node transitions (may be partial).')
            for name in (self.nav2_nodes if command == ManageLifecycleNodes.Request.PAUSE
                         else reversed(self.nav2_nodes)):
                self._transition(name, fallback_transition)
            return
        req = ManageLifecycleNodes.Request()
        req.command = command
        future = self.lifecycle_mgr_client.call_async(req)

        def _on_done(fut, cmd=command):
            try:
                res = fut.result()
            except Exception as e:
                self.get_logger().warn(f'lifecycle_manager call errored: {e}')
                return
            if res is None or not res.success:
                self.get_logger().warn(
                    f'lifecycle_manager command {cmd} reported failure.')
            else:
                label = 'PAUSE' if cmd == ManageLifecycleNodes.Request.PAUSE else 'RESUME'
                self.get_logger().info(
                    f'lifecycle_manager: {label} OK — Nav2 stack atomically updated.')

        future.add_done_callback(_on_done)

    def _transition(self, node_name, transition_id):
        client = self.change_state_clients.get(node_name)
        if client is None:
            return False
        if not client.service_is_ready():
            self.get_logger().warn(f'{node_name}/change_state not ready — skipping.')
            return False
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = client.call_async(req)

        def _on_done(fut, name=node_name, tid=transition_id):
            try:
                res = fut.result()
            except Exception as e:
                self.get_logger().warn(f'{name} transition {tid} errored: {e}')
                return
            if res is None or not res.success:
                self.get_logger().warn(f'{name} transition {tid} failed.')
            else:
                self.get_logger().info(f'{name}: transition {tid} OK.')

        future.add_done_callback(_on_done)
        return True

    def _publish_alert(self, text):
        msg = String()
        msg.data = text
        self.pub_alert.publish(msg)

    def _publish_status(self):
        msg = Bool()
        msg.data = self.emergency_stop
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StuckGuard()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

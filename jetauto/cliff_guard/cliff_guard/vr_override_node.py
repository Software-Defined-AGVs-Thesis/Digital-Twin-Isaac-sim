#!/usr/bin/env python3
"""
vr_override_node.py  —  Run INSIDE container
=============================================
Listens for VR controller commands on /vr/cmd_vel.
When human moves the thumbstick:
  - Pauses Nav2 lifecycle nodes
  - Forwards Twist to /cmd_vel to drive the robot
  - Publishes /vr_override/active = True

When human releases (timeout or service call):
  - Resumes Nav2 lifecycle nodes
  - Publishes /vr_override/active = False
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from action_msgs.srv import CancelGoal
from unique_identifier_msgs.msg import UUID
from nav2_msgs.srv import ManageLifecycleNodes

# Nav2 nodes to pause/resume — same list as cliff_guard
NAV2_LIFECYCLE_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'smoother_server',
    'velocity_smoother',
]

class VROverrideNode(Node):
    def __init__(self):
        super().__init__('vr_override')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        # When in override we ALSO publish zero on cmd_vel_nav so velocity_smoother
        # has nothing to forward (it subscribes there and re-publishes to /cmd_vel,
        # which would otherwise compete with our VR command at /cmd_vel).
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('nav_action', '/navigate_to_pose')
        self.declare_parameter('manage_nav2', True)
        self.declare_parameter('override_timeout', 3.0)  # seconds of no input before auto-release
        self.declare_parameter('lifecycle_call_timeout', 3.0)
        self.declare_parameter('nav2_nodes', NAV2_LIFECYCLE_NODES)
        # Use Nav2's lifecycle manager for atomic PAUSE/RESUME — direct per-
        # node deactivate is rejected by the manager's bonds. See cliff_guard
        # for the same fix.
        self.declare_parameter('lifecycle_manager_service',
                               '/lifecycle_manager_navigation/manage_nodes')
        # When True, ANY VR override entry engages sticky_hold automatically.
        # That makes Nav2 stay paused until you EXPLICITLY release (call the
        # /vr_override/release service, or click "Resume Autonomous" on the
        # handover popup). Set to False to fall back to the old 3-second
        # idle-timeout behavior.
        self.declare_parameter('default_sticky_hold', True)

        self.cmd_vel_topic      = self.get_parameter('cmd_vel_topic').value
        self.cmd_vel_nav_topic  = self.get_parameter('cmd_vel_nav_topic').value
        self.nav_action         = self.get_parameter('nav_action').value
        self.manage_nav2        = bool(self.get_parameter('manage_nav2').value)
        self.override_timeout   = float(self.get_parameter('override_timeout').value)
        self.lifecycle_timeout  = float(self.get_parameter('lifecycle_call_timeout').value)
        self.nav2_nodes         = list(self.get_parameter('nav2_nodes').value)
        self.lifecycle_manager_service = self.get_parameter('lifecycle_manager_service').value
        self.default_sticky     = bool(self.get_parameter('default_sticky_hold').value)

        # ── State ─────────────────────────────────────────────────────────────
        self.manual_override = False       # True when human is in control
        self.last_input_time = None        # time of last thumbstick message
        self.sticky_hold = False           # True when handover popup forces override on
                                            # — disables the idle-timeout auto-release so
                                            # a human pausing to think doesn't silently
                                            # hand control back to Nav2 next to a cliff.

        # ── Callback groups ───────────────────────────────────────────────────
        self.cb_sensors  = MutuallyExclusiveCallbackGroup()
        self.cb_services = ReentrantCallbackGroup()

        # ── Subscribers ───────────────────────────────────────────────────────
        self.sub_vr = self.create_subscription(
            Twist, '/vr/cmd_vel', self.vr_cb, 10,
            callback_group=self.cb_sensors)

        # ── Publishers ────────────────────────────────────────────────────────
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        # Used to keep /cmd_vel_nav at zero while in override, so any still-
        # active behavior_server recovery (spin/back-up) gets dropped at
        # velocity_smoother instead of being forwarded to the wheels.
        self.pub_cmd_vel_nav = self.create_publisher(Twist, self.cmd_vel_nav_topic, 10)
        self.pub_active  = self.create_publisher(Bool, '/vr_override/active', latched_qos)

        # Cancel-all client for the in-flight NavigateToPose goal. Without
        # this, when override starts and we deactivate the controller, the
        # BT immediately falls back to recovery behaviors (spin/back-up)
        # which the velocity_smoother forwards to /cmd_vel — fighting our
        # VR command. See cliff_guard for the same fix.
        self.cancel_nav_client = self.create_client(
            CancelGoal, f'{self.nav_action}/_action/cancel_goal',
            callback_group=self.cb_services)

        # ── Services ──────────────────────────────────────────────────────────
        self.srv_release = self.create_service(
            Trigger, '/vr_override/release', self.release_cb,
            callback_group=self.cb_services)
        self.srv_hold = self.create_service(
            Trigger, '/vr_override/hold', self.hold_cb,
            callback_group=self.cb_services)

        # ── Nav2 lifecycle manager client (atomic pause/resume) ───────────────
        self.lifecycle_mgr_client = self.create_client(
            ManageLifecycleNodes, self.lifecycle_manager_service,
            callback_group=self.cb_services)

        # ── Per-node lifecycle clients (fallback only) ────────────────────────
        self.change_state_clients = {}
        self.get_state_clients    = {}
        if self.manage_nav2:
            for name in self.nav2_nodes:
                self.change_state_clients[name] = self.create_client(
                    ChangeState, f'/{name}/change_state',
                    callback_group=self.cb_services)
                self.get_state_clients[name] = self.create_client(
                    GetState, f'/{name}/get_state',
                    callback_group=self.cb_services)

        # ── Timeout timer — runs every 0.5s, checks last input time ──────────
        self.timeout_timer = self.create_timer(
            0.5, self.timeout_cb,
            callback_group=self.cb_sensors)

        self.get_logger().info(
            f'VROverrideNode started — '
            f'listening on /vr/cmd_vel, '
            f'publishing to {self.cmd_vel_topic}, '
            f'timeout={self.override_timeout}s, '
            f'manage_nav2={self.manage_nav2}')

    # ── VR input callback ─────────────────────────────────────────────────────
    def vr_cb(self, msg):
        now = self.get_clock().now()

        # Check if message is non-zero (human actually moving thumbstick)
        moving = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01

        if moving:
            self.last_input_time = now

            # First time receiving input — enter override mode
            if not self.manual_override:
                self._enter_override()

            # Forward VR command to robot, AND keep zeroing /cmd_vel_nav so
            # any recovery cmd from a still-active behavior_server can't
            # reach /cmd_vel through velocity_smoother.
            self.pub_cmd_vel.publish(msg)
            self.pub_cmd_vel_nav.publish(Twist())

    # ── Timeout checker ───────────────────────────────────────────────────────
    def timeout_cb(self):
        if not self.manual_override:
            return
        if self.sticky_hold:
            # Handover popup is holding control on behalf of the human; never
            # auto-release. The popup is the only thing that can clear sticky_hold.
            return
        if self.last_input_time is None:
            return

        now     = self.get_clock().now()
        elapsed = (now - self.last_input_time).nanoseconds / 1e9  # convert to seconds

        if elapsed >= self.override_timeout:
            self.get_logger().info(
                f'No VR input for {elapsed:.1f}s — auto-releasing override.')
            self._exit_override()

    # ── Manual release service ────────────────────────────────────────────────
    def release_cb(self, request, response):
        # Always clear sticky_hold on release — popup pressed "Resume Autonomous".
        self.sticky_hold = False
        if not self.manual_override:
            response.success = True
            response.message = 'Not in override mode, nothing to release.'
            return response

        self._exit_override()
        response.success = True
        response.message = 'VR override released — Nav2 resumed.'
        return response

    # ── Sticky hold service (handover popup) ──────────────────────────────────
    def hold_cb(self, request, response):
        # Latches override ON without requiring thumbstick input. Used by the
        # handover popup after the human accepts manual control following a
        # cliff detection. Idempotent.
        self.sticky_hold = True
        if not self.manual_override:
            self._enter_override()
        # Refresh last_input_time so any concurrent timeout check is harmless
        # if sticky_hold gets cleared between checks.
        self.last_input_time = self.get_clock().now()
        response.success = True
        response.message = 'VR override held — Nav2 paused, awaiting human control.'
        return response

    # ── Enter override ────────────────────────────────────────────────────────
    def _enter_override(self):
        self.manual_override = True
        # Engage sticky_hold by default so Nav2 stays paused until the user
        # explicitly clicks Resume Autonomous (or calls /vr_override/release).
        # Without this, after 3s of thumbstick idle vr_override silently
        # auto-releases — meaning Nav2 wakes up while you're thinking.
        if self.default_sticky and not self.sticky_hold:
            self.sticky_hold = True
            self.get_logger().info(
                'sticky_hold engaged by default — Nav2 will stay paused until '
                'an explicit /vr_override/release call.')
        self.get_logger().info('VR OVERRIDE ACTIVE — human in control, Nav2 paused.')
        # 1. Cancel the in-flight Nav2 goal FIRST. This makes bt_navigator
        #    stop ticking the BT, so recoveries don't fire when the
        #    controller goes inactive in step 3.
        self._cancel_nav_goals()
        # 2. Override velocity_smoother's input briefly so any in-flight
        #    recovery cmd that's already in the smoother's queue gets
        #    flushed by zeros. We also call this every override-tick (see
        #    vr_cb) so the smoother never wins against the VR command.
        self.pub_cmd_vel_nav.publish(Twist())
        # 3. Best-effort lifecycle pause for cleanup (some transitions
        #    may still fail; that's OK — steps 1 and 2 already silenced
        #    the smoother's contribution to /cmd_vel).
        if self.manage_nav2:
            self._pause_nav2()
        self._publish_active(True)

    def _cancel_nav_goals(self):
        # action_msgs/srv/CancelGoal: zero goal_id + zero stamp = "cancel
        # every active goal on this action server".
        if not self.cancel_nav_client.service_is_ready():
            self.get_logger().warn(
                f'{self.nav_action}/_action/cancel_goal not ready — skipping cancel.')
            return
        req = CancelGoal.Request()
        req.goal_info.goal_id = UUID()           # 16 zero bytes
        req.goal_info.stamp.sec = 0
        req.goal_info.stamp.nanosec = 0
        self.cancel_nav_client.call_async(req)   # fire-and-forget
        self.get_logger().info(f'Cancel-all dispatched to {self.nav_action}.')

    # ── Exit override ─────────────────────────────────────────────────────────
    def _exit_override(self):
        # Stop robot before handing back to Nav2
        self.pub_cmd_vel.publish(Twist())
        if self.manage_nav2:
            self._resume_nav2()
        self.manual_override  = False
        self.last_input_time  = None
        self.get_logger().info('VR OVERRIDE RELEASED — Nav2 resumed, autonomous mode.')
        self._publish_active(False)

    # ── Nav2 pause/resume via lifecycle_manager (atomic) ──────────────────────
    # Direct per-node deactivate is rejected by Nav2's lifecycle_manager (which
    # owns the bonded nodes), causing partial pause and the robot to keep
    # moving. Use the manager's own manage_nodes service for atomicity.
    def _pause_nav2(self):
        self._call_lifecycle_manager(ManageLifecycleNodes.Request.PAUSE,
                                     fallback_transition=Transition.TRANSITION_DEACTIVATE)

    def _resume_nav2(self):
        self._call_lifecycle_manager(ManageLifecycleNodes.Request.RESUME,
                                     fallback_transition=Transition.TRANSITION_ACTIVATE)

    def _call_lifecycle_manager(self, command, fallback_transition):
        if not self.lifecycle_mgr_client.service_is_ready():
            self.get_logger().warn(
                f'{self.lifecycle_manager_service} not ready — falling back '
                f'to per-node transitions (may be partial).')
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

    # ── Lifecycle transition (non-blocking) ───────────────────────────────────
    # Called from inside service callbacks (release_cb, hold_cb). Must NOT block
    # with spin_until_future_complete — that deadlocks on the executor and was
    # what made /vr_override/release hang. Fire-and-forget; the executor will
    # process the response on another thread and log via the done callback.
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

    # ── Get Nav2 node state ───────────────────────────────────────────────────
    def _get_state_id(self, node_name):
        client = self.get_state_clients.get(node_name)
        if client is None:
            return State.PRIMARY_STATE_UNKNOWN
        if not client.wait_for_service(timeout_sec=self.lifecycle_timeout):
            return State.PRIMARY_STATE_UNKNOWN
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.lifecycle_timeout)
        if not future.done() or future.result() is None:
            return State.PRIMARY_STATE_UNKNOWN
        return future.result().current_state.id

    # ── Publish active status ─────────────────────────────────────────────────
    def _publish_active(self, active: bool):
        msg = Bool()
        msg.data = active
        self.pub_active.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VROverrideNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        # Make sure robot stops and Nav2 resumes if node is killed
        node.pub_cmd_vel.publish(Twist())
        if node.manual_override and node.manage_nav2:
            node._resume_nav2()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
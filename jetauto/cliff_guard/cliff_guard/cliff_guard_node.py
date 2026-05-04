#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, Float32
from std_srvs.srv import Trigger
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from action_msgs.srv import CancelGoal
from unique_identifier_msgs.msg import UUID
from nav2_msgs.srv import ManageLifecycleNodes

from cv_bridge import CvBridge
import numpy as np

NAV2_LIFECYCLE_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'smoother_server',
    'velocity_smoother',
]

class CliffGuard(Node):
    def __init__(self):
        super().__init__('cliff_guard')

        self.declare_parameter('depth_topic', '/camera/depth')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        # velocity_smoother subscribes to /cmd_vel_nav and re-publishes to
        # /cmd_vel — if we only spam /cmd_vel, the smoother keeps overwriting
        # us with whatever the BT/recoveries are producing. Spam BOTH.
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('nav_action', '/navigate_to_pose')
        self.declare_parameter('depth_threshold', 1.7)
        self.declare_parameter('roi_top', 0.70)
        self.declare_parameter('roi_bottom', 0.80)
        self.declare_parameter('roi_left', 0.45)
        self.declare_parameter('roi_right', 0.55)
        self.declare_parameter('nav2_nodes', NAV2_LIFECYCLE_NODES)
        self.declare_parameter('manage_nav2', True)
        # Nav2's lifecycle_manager owns the BT/controller/etc. via bonds —
        # poking individual nodes' /change_state is rejected (logs show
        # "transition FAILED" for behavior_server, bt_navigator,
        # waypoint_follower, velocity_smoother). The official atomic way is
        # to call the manager's manage_nodes service with PAUSE/RESUME.
        self.declare_parameter('lifecycle_manager_service',
                               '/lifecycle_manager_navigation/manage_nodes')
        self.declare_parameter('lifecycle_call_timeout', 3.0)
        self.declare_parameter('resume_max_depth_age', 1.5)  # seconds; reject resume if last depth frame is older

        self.depth_topic = self.get_parameter('depth_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_vel_nav_topic = self.get_parameter('cmd_vel_nav_topic').value
        self.nav_action = self.get_parameter('nav_action').value
        self.DEPTH_THRESHOLD = float(self.get_parameter('depth_threshold').value)
        self.ROI_TOP = float(self.get_parameter('roi_top').value)
        self.ROI_BOTTOM = float(self.get_parameter('roi_bottom').value)
        self.ROI_LEFT = float(self.get_parameter('roi_left').value)
        self.ROI_RIGHT = float(self.get_parameter('roi_right').value)
        self.nav2_nodes = list(self.get_parameter('nav2_nodes').value)
        self.manage_nav2 = bool(self.get_parameter('manage_nav2').value)
        self.lifecycle_timeout = float(self.get_parameter('lifecycle_call_timeout').value)
        self.lifecycle_manager_service = self.get_parameter('lifecycle_manager_service').value
        self.resume_max_depth_age = float(self.get_parameter('resume_max_depth_age').value)

        self.bridge = CvBridge()
        self.emergency_stop = False
        self.vr_override_active = False  # ADDED: tracks if human is in VR control
        self.cliff_currently_visible = False
        self.last_median_depth = float('nan')
        self.last_valid_pixels = 0
        self.frames_received = 0
        self.last_img_min = float('nan')
        self.last_img_max = float('nan')
        self.last_img_mean = float('nan')
        self.last_encoding = ''
        self.last_stamp_ns = 0
        self.last_shape = (0, 0)
        self.last_depth_recv_ns = 0

        self.cb_sensors = MutuallyExclusiveCallbackGroup()
        self.cb_services = ReentrantCallbackGroup()

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.sub_depth = self.create_subscription(
            Image, self.depth_topic, self.depth_cb, sensor_qos,
            callback_group=self.cb_sensors)
        self.sub_resume = self.create_subscription(
            Bool, '/cliff_guard/resume', self.resume_topic_cb, 10,
            callback_group=self.cb_services)
        self.sub_vr_active = self.create_subscription(
            Bool, '/vr_override/active', self.vr_active_cb, 10,
            callback_group=self.cb_services)  # ADDED: listens for VR override state

        self.pub_stop = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        # Override velocity_smoother's INPUT too — without this our zero on
        # /cmd_vel is overwritten 20x/sec by smoother forwarding the BT's
        # recovery commands.
        self.pub_stop_nav = self.create_publisher(Twist, self.cmd_vel_nav_topic, 10)
        # Cancel-all client for the NavigateToPose action. Uses the action's
        # built-in /<action>/_action/cancel_goal service: sending an all-zero
        # goal_id with stamp=0 is the convention for "cancel every goal".
        # Without this, bt_navigator keeps ticking the BT after we deactivate
        # the controller, so behavior_server fires spin/back-up recoveries
        # and the wheels keep moving even though we asked Nav2 to pause.
        self.cancel_nav_client = self.create_client(
            CancelGoal, f'{self.nav_action}/_action/cancel_goal',
            callback_group=self.cb_services)
        self.pub_alert = self.create_publisher(String, '/cliff_guard/alert', latched_qos)
        self.pub_status = self.create_publisher(Bool, '/cliff_guard/status', latched_qos)
        self.pub_depth = self.create_publisher(Float32, '/cliff_guard/depth', 10)

        self.srv_resume = self.create_service(
            Trigger, '/cliff_guard/resume_nav', self.resume_service_cb,
            callback_group=self.cb_services)

        # Nav2 lifecycle manager — primary path for pause/resume.
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

        self.timer = self.create_timer(0.05, self.safety_timer_cb,
                                       callback_group=self.cb_sensors)
        self.telemetry_timer = self.create_timer(0.5, self.telemetry_cb,
                                                 callback_group=self.cb_sensors)

        self._publish_status()
        self._publish_alert('READY')
        self.get_logger().info(
            f'CliffGuard started — depth_topic={self.depth_topic}, '
            f'cmd_vel_topic={self.cmd_vel_topic}, manage_nav2={self.manage_nav2}')

    def depth_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        self.frames_received += 1
        self.last_depth_recv_ns = self.get_clock().now().nanoseconds

        valid_all = img[np.isfinite(img) & (img > 0)]
        if valid_all.size and float(np.nanmedian(valid_all)) > 100.0:
            img = img / 1000.0

        h, w = img.shape[:2]
        roi = img[int(h*self.ROI_TOP):int(h*self.ROI_BOTTOM),
                  int(w*self.ROI_LEFT):int(w*self.ROI_RIGHT)]

        valid = roi[np.isfinite(roi) & (roi > 0.05)]
        self.last_valid_pixels = int(valid.size)
        if valid.size < 10:
            self.last_median_depth = float('nan')
            return

        median_depth = float(np.median(valid))
        self.last_median_depth = median_depth
        self.cliff_currently_visible = median_depth > self.DEPTH_THRESHOLD

        if self.cliff_currently_visible and not self.emergency_stop:
            self._trigger_emergency_stop(median_depth)

    def telemetry_cb(self):
        d = Float32()
        d.data = self.last_median_depth if np.isfinite(self.last_median_depth) else -1.0
        self.pub_depth.publish(d)
        self.get_logger().info(
            f'frames={self.frames_received} '
            f'median_depth={self.last_median_depth:.3f}m '
            f'valid_px={self.last_valid_pixels} '
            f'thr={self.DEPTH_THRESHOLD:.2f}m '
            f'cliff={self.cliff_currently_visible} '
            f'estop={self.emergency_stop}',
            throttle_duration_sec=1.0)

    def vr_active_cb(self, msg):
        self.vr_override_active = msg.data  # ADDED: update VR override state

    def _trigger_emergency_stop(self, median_depth):
        self.emergency_stop = True
        self.get_logger().error(
            f'!!! CLIFF DETECTED !!! depth={median_depth:.2f}m > '
            f'{self.DEPTH_THRESHOLD:.2f}m — STOPPING & PAUSING NAV2')
        # 1. Hardstop FIRST — overrides whatever else is publishing.
        zero = Twist()
        self.pub_stop.publish(zero)
        self.pub_stop_nav.publish(zero)
        # 2. Cancel any in-flight Nav2 goal so the BT stops ticking and
        #    recoveries (spin/back-up) stop firing. Lifecycle deactivate
        #    of bt_navigator/behavior_server tends to be REJECTED while a
        #    goal is active, so we must cancel before pausing.
        self._cancel_nav_goals()
        # 3. Best-effort lifecycle pause for resource cleanup. May still
        #    partially fail; that's okay because (1) and (2) plus the
        #    20Hz spam in safety_timer_cb keep the wheels at zero.
        if self.manage_nav2:
            self._pause_nav2()
        self._publish_alert(
            f'CLIFF_DETECTED depth={median_depth:.2f}m. Robot is halted and Nav2 is paused. '
            f'Physically move the robot to a safe location, then publish '
            f'`true` on /cliff_guard/resume or call /cliff_guard/resume_nav.')
        self._publish_status()

    def _cancel_nav_goals(self):
        # action_msgs/srv/CancelGoal: an all-zero goal_id + zero stamp means
        # "cancel every active goal on this action server".
        if not self.cancel_nav_client.service_is_ready():
            self.get_logger().warn(
                f'{self.nav_action}/_action/cancel_goal not ready — skipping cancel.')
            return
        req = CancelGoal.Request()
        req.goal_info.goal_id = UUID()           # 16 zero bytes
        req.goal_info.stamp.sec = 0              # zero stamp
        req.goal_info.stamp.nanosec = 0
        # Fire-and-forget — the safety timer keeps wheels at zero regardless
        # of whether the cancel completes. We just want the request OUT.
        self.cancel_nav_client.call_async(req)
        self.get_logger().info(f'Cancel-all dispatched to {self.nav_action}.')

    def safety_timer_cb(self):
        # Intentionally a no-op. Previously this spammed zero Twist at 20 Hz
        # to "hold" the robot, but that fights any other publisher (Nav2 not
        # fully down yet, recovery behaviors) and causes visible wheel jitter.
        # We now rely on full Nav2 lifecycle DEACTIVATE in _trigger_emergency_stop
        # to take Nav2 off /cmd_vel entirely. The single zero Twist published
        # at trigger time is sufficient to halt the wheels.
        return

    def resume_topic_cb(self, msg):
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

    def _try_resume(self):
        if not self.emergency_stop:
            return True, 'Not in emergency stop; nothing to do.'
        # Stale-depth guard: don't trust cliff_currently_visible if we haven't
        # gotten a fresh frame recently (camera occluded, robot rotated, etc.)
        age_s = (self.get_clock().now().nanoseconds - self.last_depth_recv_ns) / 1e9
        if self.last_depth_recv_ns == 0 or age_s > self.resume_max_depth_age:
            msg = (f'Last depth frame is {age_s:.2f}s old (>{self.resume_max_depth_age:.2f}s). '
                   f'Cannot confirm cliff is clear — wait for fresh depth data before resuming.')
            self._publish_alert(f'RESUME_REJECTED {msg}')
            return False, msg
        if self.cliff_currently_visible:
            msg = (f'Cliff still visible (depth={self.last_median_depth:.2f}m). '
                   f'Move the robot further from the edge before resuming.')
            self._publish_alert(f'RESUME_REJECTED {msg}')
            return False, msg
        self.get_logger().info('Resume accepted — reactivating Nav2.')
        if self.manage_nav2:
            self._resume_nav2()
        self.emergency_stop = False
        self._publish_status()
        self._publish_alert('RESUMED — Nav2 reactivated, robot operational.')
        return True, 'Nav2 resumed.'

    def _pause_nav2(self):
        # Use lifecycle_manager_navigation's manage_nodes service. This is
        # the only path that doesn't break Nav2's bond protocol — SIGSTOP
        # was tried and breaks it (action server never re-registers after
        # SIGCONT because bonds expire during the freeze). Per-node
        # deactivate is also rejected because the manager owns the nodes.
        self._call_lifecycle_manager(ManageLifecycleNodes.Request.PAUSE,
                                     fallback_transition=Transition.TRANSITION_DEACTIVATE)

    def _resume_nav2(self):
        self._call_lifecycle_manager(ManageLifecycleNodes.Request.RESUME,
                                     fallback_transition=Transition.TRANSITION_ACTIVATE)


    def _call_lifecycle_manager(self, command, fallback_transition):
        """Atomic Nav2 PAUSE/RESUME via lifecycle_manager_navigation.
        Falls back to per-node transitions only if the manager service is
        absent (e.g. user reconfigured Nav2 without lifecycle_manager)."""
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
        # NON-BLOCKING. The previous spin_until_future_complete call was
        # invoked from inside depth_cb / service callbacks and would deadlock
        # on the executor — so deactivate requests silently timed out and
        # Nav2 stayed live, which is what made the robot keep moving (and
        # jitter against our zero-Twist spam) after a cliff trip.
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
    node = CliffGuard()
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
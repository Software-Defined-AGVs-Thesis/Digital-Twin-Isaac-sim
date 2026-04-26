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
        self.declare_parameter('depth_threshold', 1.7)
        self.declare_parameter('roi_top', 0.70)
        self.declare_parameter('roi_bottom', 0.80)
        self.declare_parameter('roi_left', 0.45)
        self.declare_parameter('roi_right', 0.55)
        self.declare_parameter('nav2_nodes', NAV2_LIFECYCLE_NODES)
        self.declare_parameter('manage_nav2', True)
        self.declare_parameter('lifecycle_call_timeout', 3.0)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.DEPTH_THRESHOLD = float(self.get_parameter('depth_threshold').value)
        self.ROI_TOP = float(self.get_parameter('roi_top').value)
        self.ROI_BOTTOM = float(self.get_parameter('roi_bottom').value)
        self.ROI_LEFT = float(self.get_parameter('roi_left').value)
        self.ROI_RIGHT = float(self.get_parameter('roi_right').value)
        self.nav2_nodes = list(self.get_parameter('nav2_nodes').value)
        self.manage_nav2 = bool(self.get_parameter('manage_nav2').value)
        self.lifecycle_timeout = float(self.get_parameter('lifecycle_call_timeout').value)

        self.bridge = CvBridge()
        self.emergency_stop = False
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

        self.pub_stop = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_alert = self.create_publisher(String, '/cliff_guard/alert', latched_qos)
        self.pub_status = self.create_publisher(Bool, '/cliff_guard/status', latched_qos)
        self.pub_depth = self.create_publisher(Float32, '/cliff_guard/depth', 10)

        self.srv_resume = self.create_service(
            Trigger, '/cliff_guard/resume_nav', self.resume_service_cb,
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

    def _trigger_emergency_stop(self, median_depth):
        self.emergency_stop = True
        self.get_logger().error(
            f'!!! CLIFF DETECTED !!! depth={median_depth:.2f}m > '
            f'{self.DEPTH_THRESHOLD:.2f}m — STOPPING & PAUSING NAV2')
        self.pub_stop.publish(Twist())
        if self.manage_nav2:
            self._pause_nav2()
        self._publish_alert(
            f'CLIFF_DETECTED depth={median_depth:.2f}m. Robot is halted and Nav2 is paused. '
            f'Physically move the robot to a safe location, then publish '
            f'`true` on /cliff_guard/resume or call /cliff_guard/resume_nav.')
        self._publish_status()

    def safety_timer_cb(self):
        if self.emergency_stop:
            self.pub_stop.publish(Twist())

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
        for name in self.nav2_nodes:
            self._transition(name, Transition.TRANSITION_DEACTIVATE)

    def _resume_nav2(self):
        for name in reversed(self.nav2_nodes):
            state_id = self._get_state_id(name)
            if state_id == State.PRIMARY_STATE_ACTIVE:
                continue
            if state_id == State.PRIMARY_STATE_INACTIVE:
                self._transition(name, Transition.TRANSITION_ACTIVATE)

    def _transition(self, node_name, transition_id):
        client = self.change_state_clients.get(node_name)
        if client is None:
            return False
        if not client.wait_for_service(timeout_sec=self.lifecycle_timeout):
            self.get_logger().warn(f'{node_name}/change_state unavailable — skipping.')
            return False
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.lifecycle_timeout)
        if not future.done():
            self.get_logger().warn(f'{node_name} transition timed out.')
            return False
        result = future.result()
        if result is None or not result.success:
            self.get_logger().warn(f'{node_name} transition {transition_id} failed.')
            return False
        self.get_logger().info(f'{node_name}: transition {transition_id} OK.')
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

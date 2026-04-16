import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from omni.isaac.core.articulations import Articulation
import omni.kit.app
import omni.timeline
import numpy as np


class JetAutoController(Node):
    def __init__(self, robot_prim_path):
        super().__init__('jetauto_controller')
    
        self.robot_prim_path = robot_prim_path
        self.robot = None
        self._timeline = omni.timeline.get_timeline_interface()

        # JetAuto mecanum wheel parameters
        self.wheel_radius = 0.049
        self.lx = 0.1125
        self.ly = 0.1165

        self.wheel_vels = np.zeros(4)

        self._play_started = False
        self._init_delay = 0
        self._init_delay_target = 60

        self.wheel_joint_names = [
            "wheel_left_front_joint",
            "wheel_right_front_joint",
            "wheel_left_back_joint",
            "wheel_right_back_joint",
        ]
        self.wheel_dof_indices = None

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)

        print("[JetAuto] Controller created!")

    def try_initialize(self):
        if self.robot is None:
            try:
                self.robot = Articulation(self.robot_prim_path)
                self.robot.initialize()

                dof_names = self.robot.dof_names
                print(f"[JetAuto] Initialized! DOF names: {dof_names}")
                print(f"[JetAuto] Num DOFs: {self.robot.num_dof}")

                self.wheel_dof_indices = []
                for wheel_name in self.wheel_joint_names:
                    matched = False
                    for i, dof in enumerate(dof_names):
                        if wheel_name in dof or dof in wheel_name:
                            self.wheel_dof_indices.append(i)
                            matched = True
                            print(f"[JetAuto] Mapped {wheel_name} → DOF index {i} ({dof})")
                            break
                    if not matched:
                        print(f"[JetAuto] WARNING: Could not find DOF for {wheel_name}")

                if len(self.wheel_dof_indices) != 4:
                    print("[JetAuto] ERROR: Could not map all 4 wheels. DOF names above.")
                    self.robot = None
                    return False

                return True

            except Exception as e:
                print(f"[JetAuto] Init failed: {e}")
                self.robot = None
                return False
        return True

    def cmd_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        r = self.wheel_radius
        k = self.lx + self.ly

        fl = (vx - vy - k * wz) / r
        fr = (vx + vy + k * wz) / r
        rl = (vx + vy - k * wz) / r
        rr = (vx - vy + k * wz) / r

        self.wheel_vels = np.array([fl, fr, rl, rr])

    def update(self):
        is_playing = self._timeline.is_playing()

        if is_playing and not self._play_started:
            self._play_started = True
            self._init_delay = 0
            self.robot = None
            self.wheel_dof_indices = None
            print("[JetAuto] Simulation started - waiting for physics...")

        if not is_playing:
            self._play_started = False
            return

        if self._init_delay < self._init_delay_target:
            self._init_delay += 1
            return

        if not self.try_initialize():
            return

        if self.robot is not None and self.wheel_dof_indices is not None:
            try:
                full_vels = np.zeros(self.robot.num_dof)
                for i, dof_idx in enumerate(self.wheel_dof_indices):
                    full_vels[dof_idx] = self.wheel_vels[i]
                self.robot.set_joint_velocities(full_vels)
            except Exception as e:
                print(f"[JetAuto] Velocity error: {e}")
                self.robot = None
                self.wheel_dof_indices = None


# =============================================================================
# Global state
# =============================================================================
_controller = None
_update_sub = None


def start_controller(robot_prim_path="/Robots/jetauto/jetauto"):
    global _controller, _update_sub

    if _controller is not None:
        stop_controller()

    if not rclpy.ok():
        rclpy.init()

    _controller = JetAutoController(robot_prim_path)

    def on_update(event):
        rclpy.spin_once(_controller, timeout_sec=0)
        _controller.update()

    _update_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_update)

    print("\n" + "=" * 60)
    print("  JetAuto Mecanum Controller Running!")
    print("  Robot path: " + robot_prim_path)
    print("  ")
    print("  In another terminal:")
    print("    docker exec -it isaac-sim-omnilrs-container bash")
    print("    ros2 run teleop_twist_keyboard teleop_twist_keyboard")
    print("  ")
    print("  i=forward  ,=back  j=rotate-left  l=rotate-right  k=stop")
    print("=" * 60 + "\n")

    return _controller


def stop_controller():
    global _controller, _update_sub
    _update_sub = None
    if _controller:
        _controller.destroy_node()
        _controller = None
    print("[JetAuto] Controller stopped")
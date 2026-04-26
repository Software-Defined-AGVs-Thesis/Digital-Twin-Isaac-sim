# NEW imports only — originals untouched
import math
import time
import traceback
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from omni.isaac.sensor import RotatingLidarPhysX as LidarSensor
from omni.isaac.sensor import Camera
from pxr import UsdGeom, Gf, Sdf
import omni.usd
import omni.kit.commands
import carb

# ORIGINAL imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from omni.isaac.core.articulations import Articulation
import omni.kit.app
import omni.timeline
import numpy as np


# =============================================================================
# NEW: confirmed prim paths and sensor config
# =============================================================================
LIDAR_SENSOR_PATH = "/Robots/jetauto/lidar_link/lidar_sim_frame/lidar_sensor"
ROBOT_PRIM_PATH   = "/Robots/jetauto/base_link"

LIDAR_FOV_DEG   = 360.0
LIDAR_RES_DEG   = 0.5
LIDAR_MIN_RANGE = 0.12
LIDAR_MAX_RANGE = 12.0
LIDAR_HZ        = 10.0

FRAME_ODOM           = "odom"
FRAME_BASE_FOOTPRINT = "base_footprint"
FRAME_BASE_LINK      = "base_link"
FRAME_LIDAR          = "lidar_sim_frame"
FRAME_CAMERA         = "camera_link"

BASE_LINK_Z    = 0.065
LIDAR_OFFSET_X = 0.00
LIDAR_OFFSET_Z = 0.12


# =============================================================================
# NEW: camera config
# =============================================================================
# Script creates this prim itself — no manual Create → Camera needed.
CAMERA_DST_PATH = "/Robots/jetauto/screen_link/camera_sensor"
CAMERA_PARENT   = "/Robots/jetauto/screen_link"

CAMERA_WIDTH    = 320
CAMERA_HEIGHT   = 240
# None = let Isaac use the render rate (safe default, always divides evenly).
# Set to an integer (e.g. 30) only if it cleanly divides the render rate.
CAMERA_HZ       = None
CAMERA_FOCAL_MM = 24.0
CAMERA_HAPER_MM = 20.955
CAMERA_VAPER_MM = 15.2908
CAMERA_NEAR     = 0.1
CAMERA_FAR      = 100.0

# Camera offset relative to screen_link (local, Isaac Y-up / sim frame)
#Anwar change x, y, z for depth camera
CAMERA_OFFSET_X = 0.0
CAMERA_OFFSET_Y = 0.4
CAMERA_OFFSET_Z = 0.45

# Static TF offset: base_link → camera_link (ROS frame, Z-up)
CAMERA_TF_X = 0.10
CAMERA_TF_Y = 0.00
CAMERA_TF_Z = 0.18

# Give up camera init after this many failures (stops __del__ spam)
CAMERA_MAX_ATTEMPTS = 3


# =============================================================================
# NEW: small helpers
# =============================================================================
def _stamp(node):
    from builtin_interfaces.msg import Time
    t = node.get_clock().now().nanoseconds
    msg = Time()
    msg.sec     = t // 1_000_000_000
    msg.nanosec = t %  1_000_000_000
    return msg

def _yaw_to_quat(yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return 0.0, 0.0, sy, cy  # x, y, z, w

def _make_tf(parent, child, tx, ty, tz, qx, qy, qz, qw, stamp):
    ts = TransformStamped()
    ts.header.stamp    = stamp
    ts.header.frame_id = parent
    ts.child_frame_id  = child
    ts.transform.translation.x = tx
    ts.transform.translation.y = ty
    ts.transform.translation.z = tz
    ts.transform.rotation.x = qx
    ts.transform.rotation.y = qy
    ts.transform.rotation.z = qz
    ts.transform.rotation.w = qw
    return ts


# =============================================================================
# ORIGINAL class — nothing removed, additions marked NEW
# =============================================================================
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

        # NEW: static TF republish timer
        self._static_tf_timer = 0
        self._static_tf_interval = 100

        # NEW: publishers
        sensor_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.VOLATILE,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 5,
        )
        reliable_qos = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            durability  = DurabilityPolicy.VOLATILE,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 10,
        )
        transient_qos = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            durability  = DurabilityPolicy.TRANSIENT_LOCAL,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1,
        )
        self._pub_scan      = self.create_publisher(LaserScan, '/scan',      sensor_qos)
        self._pub_odom      = self.create_publisher(Odometry,  '/odom',      reliable_qos)
        self._pub_tf        = self.create_publisher(TFMessage, '/tf',        reliable_qos)
        self._pub_tf_static = self.create_publisher(TFMessage, '/tf_static', transient_qos)
        self._pub_clock     = self.create_publisher(Clock,     '/clock',     reliable_qos)

        # NEW: camera publishers
        self._pub_image       = self.create_publisher(Image,      '/camera/image_raw',   sensor_qos)
        self._pub_depth       = self.create_publisher(Image,      '/camera/depth',       sensor_qos)
        self._pub_camera_info = self.create_publisher(CameraInfo, '/camera/camera_info', sensor_qos)

        # NEW: LiDAR state
        self._lidar       = None
        self._lidar_ready = False
        self._num_pts     = int(LIDAR_FOV_DEG / LIDAR_RES_DEG)

        # NEW: camera state
        self._camera          = None
        self._camera_ready    = False
        self._camera_created  = False   # USD prim exists
        self._camera_attempts = 0
        self._camera_gave_up  = False

        # NEW: odometry state
        self._odom_vx  = 0.0
        self._odom_vy  = 0.0
        self._odom_wz  = 0.0
        self._prev_x   = None
        self._prev_y   = None
        self._prev_yaw = None
        self._prev_t   = None
        self._frame_count = 0

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

                try:
                    self._publish_static_tf()
                except Exception as e:
                    print(f"[JetAuto] Static TF warning (non-fatal): {e}")

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
            # NEW: reset sensor state on each new play
            self._lidar = None
            self._lidar_ready = False
            self._camera = None
            self._camera_ready = False
            self._camera_attempts = 0
            self._camera_gave_up = False
            # note: _camera_created stays true across plays (USD prim persists)
            self._prev_x = self._prev_y = self._prev_yaw = self._prev_t = None
            print("[JetAuto] Simulation started - waiting for physics...")

        if not is_playing:
            self._play_started = False
            return

        if self._init_delay < self._init_delay_target:
            self._init_delay += 1
            return

        if not self.try_initialize():
            return

        self._frame_count += 1

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

        if not self._lidar_ready:
            self._try_init_lidar()

        if not self._camera_ready and not self._camera_gave_up:
            self._try_init_camera()

        self._static_tf_timer += 1
        if self._static_tf_timer % self._static_tf_interval == 0:
            try:
                self._publish_static_tf()
            except Exception:
                pass

        self._publish_clock()
        self._update_odom()
        if self._lidar_ready:
            self._publish_scan()
        if self._camera_ready and self._frame_count % 4 == 0:
                    self._publish_camera()  

    # =========================================================================
    # NEW methods
    # =========================================================================

    def _try_init_lidar(self):
        try:
            self._lidar = LidarSensor(
                prim_path          = LIDAR_SENSOR_PATH,
                name               = "jetauto_lidar",
                rotation_frequency = 10,
                fov                = (360.0, 0.0),
                resolution         = (0.5, 0.5),
                valid_range        = (LIDAR_MIN_RANGE, LIDAR_MAX_RANGE),
            )
            self._lidar.initialize()
            self._lidar.add_linear_depth_data_to_frame()
            self._lidar_ready = True
            print(f"[JetAuto] LiDAR ready at '{LIDAR_SENSOR_PATH}'")
        except Exception as e:
            if self._frame_count % 120 == 0:
                print(f"[JetAuto] LiDAR not ready yet: {e}")
            self._lidar = None
            self._lidar_ready = False

    def _publish_scan(self):
        try:
            data = self._lidar.get_current_frame()
        except Exception as e:
            carb.log_warn(f"[JetAuto] LiDAR read error: {e}")
            return

        ranges = None
        for key in ("linear_depth", "depth", "depthData"):
            if key in data:
                ranges = np.asarray(data[key], dtype=np.float32).flatten()
                break

        if ranges is None:
            if self._frame_count < 10:
                print(f"[JetAuto] LiDAR frame keys: {list(data.keys())}")
            ranges = np.full(self._num_pts, LIDAR_MAX_RANGE, dtype=np.float32)

        if len(ranges) != self._num_pts:
            ranges = np.resize(ranges, self._num_pts)

        bad = (ranges < LIDAR_MIN_RANGE) | np.isnan(ranges) | np.isinf(ranges)
        ranges[bad] = LIDAR_MAX_RANGE

        msg = LaserScan()
        msg.header.stamp    = _stamp(self)
        msg.header.frame_id = FRAME_LIDAR
        msg.angle_min       = -math.pi
        msg.angle_max       =  math.pi
        msg.angle_increment = math.radians(LIDAR_FOV_DEG) / self._num_pts
        msg.time_increment  = 0.0
        msg.scan_time       = 1.0 / LIDAR_HZ
        msg.range_min       = LIDAR_MIN_RANGE
        msg.range_max       = LIDAR_MAX_RANGE
        msg.ranges          = ranges.tolist()
        msg.intensities     = []
        self._pub_scan.publish(msg)

    # ---------------------- camera ----------------------

    def _create_camera_prim(self):
        """Create a USD Camera prim directly under screen_link.
        No manual Stage-panel action required."""
        if self._camera_created:
            return True

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return False

        # Already exists (from a previous run in this session)
        existing = stage.GetPrimAtPath(CAMERA_DST_PATH)
        if existing.IsValid():
            print(f"[JetAuto] Camera prim already exists at {CAMERA_DST_PATH}")
            self._camera_created = True
            return True

        # Parent must exist
        parent = stage.GetPrimAtPath(CAMERA_PARENT)
        if not parent.IsValid():
            if self._frame_count % 120 == 0:
                print(f"[JetAuto] Camera parent not found at {CAMERA_PARENT}")
            return False

        try:
            # Create a proper USD Camera prim at the desired path
            cam_geom = UsdGeom.Camera.Define(stage, Sdf.Path(CAMERA_DST_PATH))
            cam_prim = cam_geom.GetPrim()

            if not cam_prim.IsValid():
                print(f"[JetAuto] UsdGeom.Camera.Define returned invalid prim")
                return False

            # Local transform relative to screen_link (sim frame)
            xform = UsdGeom.Xformable(cam_prim)
            xform.ClearXformOpOrder()

            t_op = xform.AddTranslateOp()
            t_op.Set(Gf.Vec3d(CAMERA_OFFSET_X, CAMERA_OFFSET_Y, CAMERA_OFFSET_Z))

            # USD cameras look down -Z by default.
            # Rotate so the camera's view axis points along sim +X (robot forward)
            #Anwar change orientation angle
            r_op = xform.AddRotateXYZOp()
            r_op.Set(Gf.Vec3f(-75.0, 0.0, 0.0)) # was 75.0, now -75.0

            # ADD THIS:
            s_op = xform.AddScaleOp()
            s_op.Set(Gf.Vec3f(1.0, 1.0, -1.0))

            # Intrinsics
            cam_geom.GetFocalLengthAttr().Set(CAMERA_FOCAL_MM)
            cam_geom.GetHorizontalApertureAttr().Set(CAMERA_HAPER_MM)
            cam_geom.GetVerticalApertureAttr().Set(CAMERA_VAPER_MM)
            cam_geom.GetClippingRangeAttr().Set(Gf.Vec2f(CAMERA_NEAR, CAMERA_FAR))
        except Exception as e:
            print(f"[JetAuto] Camera prim creation failed: {e}")
            traceback.print_exc()
            return False

        # Verify
        check = stage.GetPrimAtPath(CAMERA_DST_PATH)
        type_name = check.GetTypeName()
        print(f"[JetAuto] Camera prim created at {CAMERA_DST_PATH}, type={type_name}")
        if type_name != "Camera":
            print(f"[JetAuto] WARNING: expected Camera type, got {type_name}")

        self._camera_created = True
        return True

    def _try_init_camera(self):
        """Create the USD camera prim (if needed), then wrap it in the Isaac
        Camera sensor API. Caps attempts to avoid __del__ spam."""
        if not self._create_camera_prim():
            return

        if self._camera_attempts >= CAMERA_MAX_ATTEMPTS:
            if not self._camera_gave_up:
                print(f"[JetAuto] Camera init gave up after {CAMERA_MAX_ATTEMPTS} attempts. "
                      f"Restart controller to retry.")
                self._camera_gave_up = True
            return

        self._camera_attempts += 1
        print(f"[JetAuto] Camera init attempt #{self._camera_attempts}...")

        try:
            # Only pass frequency if explicitly set (avoids render-rate divisor errors)
            kwargs = dict(
                prim_path  = CAMERA_DST_PATH,
                name       = "jetauto_camera",
                resolution = (CAMERA_WIDTH, CAMERA_HEIGHT),
            )
            if CAMERA_HZ is not None:
                kwargs["frequency"] = int(CAMERA_HZ)

            self._camera = Camera(**kwargs)
            self._camera.initialize()
            self._camera.add_distance_to_image_plane_to_frame()
            self._camera_ready = True
            print(f"[JetAuto] Camera ready at '{CAMERA_DST_PATH}'")
        except Exception as e:
            print(f"[JetAuto] Camera init FAILED: {type(e).__name__}: {e}")
            traceback.print_exc()
            self._camera = None
            self._camera_ready = False

    def _publish_camera(self):
        try:
            frame = self._camera.get_current_frame()
        except Exception as e:
            carb.log_warn(f"[JetAuto] Camera read error: {e}")
            return

        stamp = _stamp(self)

        # --- RGB ---
        rgba = frame.get("rgba")
        if rgba is not None:
            rgba_np = np.asarray(rgba, dtype=np.uint8)
            if rgba_np.ndim == 3 and rgba_np.shape[2] == 4:
                rgb_np = rgba_np[:, :, :3]
                if rgb_np.size > 0:
                    img = Image()
                    img.header.stamp    = stamp
                    img.header.frame_id = FRAME_CAMERA
                    img.height          = rgb_np.shape[0]
                    img.width           = rgb_np.shape[1]
                    img.encoding        = "rgb8"
                    img.is_bigendian    = 0
                    img.step            = img.width * 3
                    img.data            = rgb_np.tobytes()
                    self._pub_image.publish(img)

        # --- Depth ---
        depth = frame.get("distance_to_image_plane")
        if depth is not None:
            depth_np = np.asarray(depth, dtype=np.float32)
            if depth_np.ndim == 2 and depth_np.size > 0:
                dmsg = Image()
                dmsg.header.stamp    = stamp
                dmsg.header.frame_id = FRAME_CAMERA
                dmsg.height          = depth_np.shape[0]
                dmsg.width           = depth_np.shape[1]
                dmsg.encoding        = "32FC1"
                dmsg.is_bigendian    = 0
                dmsg.step            = dmsg.width * 4
                dmsg.data            = depth_np.tobytes()
                self._pub_depth.publish(dmsg)

        # --- CameraInfo ---
        fx = (CAMERA_FOCAL_MM / CAMERA_HAPER_MM) * CAMERA_WIDTH
        fy = (CAMERA_FOCAL_MM / CAMERA_VAPER_MM) * CAMERA_HEIGHT
        cx = CAMERA_WIDTH  / 2.0
        cy = CAMERA_HEIGHT / 2.0

        info = CameraInfo()
        info.header.stamp     = stamp
        info.header.frame_id  = FRAME_CAMERA
        info.height           = CAMERA_HEIGHT
        info.width            = CAMERA_WIDTH
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self._pub_camera_info.publish(info)

    # ---------------------- odom / tf / clock ----------------------

    def _update_odom(self):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            print("[JetAuto] _update_odom: stage is None")
            return
        prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
        if not prim.IsValid():
            if self._frame_count % 120 == 0:
                print(f"[JetAuto] _update_odom: prim not valid at {ROBOT_PRIM_PATH}")
                for p in stage.Traverse():
                    path = str(p.GetPath())
                    if "jetauto" in path.lower() or "robot" in path.lower():
                        print(f"  found: {path}")
            return

        xf_cache = UsdGeom.XformCache()
        world_xf = xf_cache.GetLocalToWorldTransform(prim)
        trans    = world_xf.ExtractTranslation()

        ros_x =  trans[0]
        ros_y = -trans[2]

        rot   = world_xf.ExtractRotationMatrix()
        fwd_x = rot[0][0]
        fwd_y = rot[2][0]
        yaw   = math.atan2(-fwd_y, fwd_x)

        now = time.monotonic()
        if self._prev_t is not None:
            dt = now - self._prev_t
            if dt > 1e-6:
                dx   = ros_x - self._prev_x
                dy   = ros_y - self._prev_y
                dyaw = yaw   - self._prev_yaw
                while dyaw >  math.pi: dyaw -= 2.0 * math.pi
                while dyaw < -math.pi: dyaw += 2.0 * math.pi
                cy, sy = math.cos(yaw), math.sin(yaw)
                self._odom_vx =  dx * cy + dy * sy
                self._odom_vy = -dx * sy + dy * cy
                self._odom_wz = dyaw / dt

        self._prev_x   = ros_x
        self._prev_y   = ros_y
        self._prev_yaw = yaw
        self._prev_t   = now

        qx, qy, qz, qw = _yaw_to_quat(yaw)
        stamp = _stamp(self)

        odom = Odometry()
        odom.header.stamp            = stamp
        odom.header.frame_id         = FRAME_ODOM
        odom.child_frame_id          = FRAME_BASE_FOOTPRINT
        odom.pose.pose.position.x    = ros_x
        odom.pose.pose.position.y    = ros_y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x    = self._odom_vx
        odom.twist.twist.linear.y    = self._odom_vy
        odom.twist.twist.angular.z   = self._odom_wz
        odom.pose.covariance[0]      = 0.01
        odom.pose.covariance[7]      = 0.01
        odom.pose.covariance[35]     = 0.01
        odom.twist.covariance[0]     = 0.01
        odom.twist.covariance[7]     = 0.01
        odom.twist.covariance[35]    = 0.01
        self._pub_odom.publish(odom)

        tf_msg = TFMessage()
        tf_msg.transforms = [
            _make_tf(FRAME_ODOM, FRAME_BASE_FOOTPRINT,
                     ros_x, ros_y, 0.0,
                     qx, qy, qz, qw, stamp)
        ]
        self._pub_tf.publish(tf_msg)

    def _publish_static_tf(self):
        stamp  = _stamp(self)
        tf_msg = TFMessage()
        tf_msg.transforms = [
            _make_tf(FRAME_BASE_FOOTPRINT, FRAME_BASE_LINK,
                     0.0, 0.0, BASE_LINK_Z,
                     0.0, 0.0, 0.0, 1.0, stamp),
            _make_tf(FRAME_BASE_LINK, FRAME_LIDAR,
                     LIDAR_OFFSET_X, 0.0, LIDAR_OFFSET_Z,
                     0.0, 0.0, 0.0, 1.0, stamp),
            _make_tf(FRAME_BASE_LINK, FRAME_CAMERA,
                     CAMERA_TF_X, CAMERA_TF_Y, CAMERA_TF_Z,
                     0.0, 0.0, 0.0, 1.0, stamp),
        ]
        self._pub_tf_static.publish(tf_msg)
        print("[JetAuto] Static TF published: base_footprint → base_link → {lidar_sim_frame, camera_link}")

    def _publish_clock(self):
        msg = Clock()
        msg.clock = _stamp(self)
        self._pub_clock.publish(msg)


# =============================================================================
# ORIGINAL global state — unchanged
# =============================================================================
_controller = None
_update_sub = None


def start_controller(robot_prim_path="/Robots/jetauto"):
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
    print("  ")
    print("  Topics: /scan /odom /tf /tf_static /clock")
    print("          /camera/image_raw /camera/depth /camera/camera_info")
    print("=" * 60 + "\n")

    return _controller


def stop_controller():
    global _controller, _update_sub
    _update_sub = None
    if _controller:
        _controller.destroy_node()
        _controller = None
    print("[JetAuto] Controller stopped")
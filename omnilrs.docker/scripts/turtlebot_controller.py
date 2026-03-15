import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from omni.isaac.core.articulations import Articulation
import omni.kit.app
import omni.timeline


class TurtlebotController(Node):
    def __init__(self, robot_prim_path):
        super().__init__('turtlebot_controller')
        
        self.robot_prim_path = robot_prim_path
        self.robot = None
        self._timeline = omni.timeline.get_timeline_interface()
        
        # TurtleBot3 Burger parameters
        self.wheel_radius = 0.033
        self.wheel_distance = 0.16
        
        # Store latest command
        self.left_vel = 0.0
        self.right_vel = 0.0
        
        # Delay initialization by a few frames after play
        self._play_started = False
        self._init_delay = 0
        self._init_delay_target = 60  # Wait ~1 second after play
        
        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)
        
        print("[TurtleBot] Controller created!")
    
    def try_initialize(self):
        """Try to initialize robot if not done"""
        if self.robot is None:
            try:
                self.robot = Articulation(self.robot_prim_path)
                self.robot.initialize()
                print("[TurtleBot] Robot initialized!")
                return True
            except Exception as e:
                self.robot = None
                return False
        return True
    
    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Differential drive conversion
        self.left_vel = (linear - angular * self.wheel_distance / 2) / self.wheel_radius
        self.right_vel = (linear + angular * self.wheel_distance / 2) / self.wheel_radius
        
    def update(self):
        is_playing = self._timeline.is_playing()
        
        # Detect play start - reset and wait
        if is_playing and not self._play_started:
            self._play_started = True
            self._init_delay = 0
            self.robot = None
            print("[TurtleBot] Simulation started - waiting for physics...")
        
        # Detect stop
        if not is_playing:
            self._play_started = False
            return
        
        # Wait for physics to be ready
        if self._init_delay < self._init_delay_target:
            self._init_delay += 1
            return
        
        # Try init if needed
        if not self.try_initialize():
            return
        
        # Apply velocities
        if self.robot is not None:
            try:
                self.robot.set_joint_velocities([self.left_vel, self.right_vel])
            except:
                self.robot = None  # Reset on error


# =============================================================================
# Global state
# =============================================================================

_controller = None
_update_sub = None


def start_controller(robot_prim_path="/Robots/turtlebot3_burger/turtlebot3_burger/a__namespace_base_footprint"):
    global _controller, _update_sub
    
    # Stop existing controller if running
    if _controller is not None:
        stop_controller()
    
    # Initialize ROS2 if not already done
    if not rclpy.ok():
        rclpy.init()
    
    # Create controller
    _controller = TurtlebotController(robot_prim_path)
    
    # Update function called every frame
    def on_update(event):
        rclpy.spin_once(_controller, timeout_sec=0)
        _controller.update()
    
    # Register update callback
    _update_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_update)
    
    print("\n" + "="*60)
    print("  TurtleBot Controller Running!")
    print("  ")
    print("  In another terminal, run:")
    print("    docker exec -it isaac-sim-omnilrs-container bash")
    print("    teleop")
    print("  ")
    print("  Controls: i=forward, ,=back, j=left, l=right, k=stop")
    print("="*60 + "\n")
    
    return _controller


def stop_controller():
    global _controller, _update_sub
    _update_sub = None
    if _controller:
        _controller.destroy_node()
        _controller = None
    print("[TurtleBot] Controller stopped")
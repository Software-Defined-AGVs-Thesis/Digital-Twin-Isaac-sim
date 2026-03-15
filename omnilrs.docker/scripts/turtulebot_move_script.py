# this is a tmp script to paste in the script editor in isaacsim

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from omni.isaac.core.articulations import Articulation
import omni.kit.app

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        
        # Initialize robot
        self.robot = Articulation("/Robots/turtlebot3_burger/a__namespace_base_footprint")
        self.robot.initialize()
        
        # TurtleBot3 Burger parameters
        self.wheel_radius = 0.033
        self.wheel_distance = 0.16
        
        # Store latest command
        self.left_vel = 0.0
        self.right_vel = 0.0
        
        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)
        
        print("TurtleBot controller initialized!")
    
    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Differential drive conversion
        self.left_vel = (linear - angular * self.wheel_distance / 2) / self.wheel_radius
        self.right_vel = (linear + angular * self.wheel_distance / 2) / self.wheel_radius
        
    def update(self):
        # Apply velocities to robot
        self.robot.set_joint_velocities([self.left_vel, self.right_vel])


# Only init if not already initialized
if not rclpy.ok():
    rclpy.init()

controller = TurtlebotController()

# Update function called every frame
def on_update(event):
    rclpy.spin_once(controller, timeout_sec=0)
    controller.update()

# Register update callback
update_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_update)

print("Controller running! Use teleop_twist_keyboard to control.")
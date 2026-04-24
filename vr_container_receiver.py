#!/usr/bin/env python3
"""
vr_container_receiver.py  —  Run INSIDE container
===================================================
Receives joystick data via UDP from host and
publishes geometry_msgs/Twist to /cmd_vel.

Usage:
    source /opt/ros/humble/setup.bash
    python3 vr_container_receiver.py
"""

import socket
import json
import sys

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
except ImportError:
    print("ERROR: rclpy not found. Source ROS2 first:")
    print("  source /opt/ros/humble/setup.bash")
    sys.exit(1)

# ── Tuning ────────────────────────────────────────────────────────────────────
UDP_IP      = "0.0.0.0"
UDP_PORT    = 5005
MAX_LINEAR  = 0.5   # m/s
MAX_ANGULAR = 1.0   # rad/s
RATE_HZ     = 20
# ─────────────────────────────────────────────────────────────────────────────


class VRUDPReceiver(Node):
    def __init__(self):
        super().__init__('vr_udp_receiver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.settimeout(0.1)

        self.timer = self.create_timer(1.0 / RATE_HZ, self.timer_callback)

        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Listening for joystick data on UDP port {UDP_PORT}")
        self.get_logger().info("Publishing to /cmd_vel")
        self.get_logger().info("=" * 50)

    def timer_callback(self):
        try:
            data_bytes, _ = self.sock.recvfrom(1024)
            data = json.loads(data_bytes.decode())
        except socket.timeout:
            # No new data — publish zero to stop if host dies
            self.pub.publish(Twist())
            return
        except Exception as e:
            self.get_logger().warn(f"Bad packet: {e}")
            return

        msg = Twist()
        if not data.get("stop", False):
            msg.linear.x  =  data.get("y", 0.0) * MAX_LINEAR
            msg.angular.z = -data.get("x", 0.0) * MAX_ANGULAR

        self.pub.publish(msg)

        if abs(msg.linear.x) > 0 or abs(msg.angular.z) > 0:
            self.get_logger().info(
                f"linear.x={msg.linear.x:.2f}  angular.z={msg.angular.z:.2f}",
                throttle_duration_sec=0.2
            )


def main():
    rclpy.init()
    node = VRUDPReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
        print("\nStopped. Robot halted.")
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
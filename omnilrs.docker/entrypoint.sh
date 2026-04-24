#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

# Fix spdlog conflict: system and ROS2 libs must come before Isaac Sim's bundled libs
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# Isaac Sim ROS2 bridge libraries
export LD_LIBRARY_PATH=/isaac-sim/exts/omni.isaac.ros2_bridge/humble/lib:$LD_LIBRARY_PATH

# ROS2 middleware implementation
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

cd /workspace/omnilrs
exec "$@"

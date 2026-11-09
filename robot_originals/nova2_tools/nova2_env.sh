#!/usr/bin/env bash
# Source this file for Nova2 ROS2 / MoveIt environment.
source /opt/ros/humble/setup.bash
source /home/nvidia/nova2_ros2_ws/install/local_setup.bash
export DOBOT_TYPE=nova2
export IP_address=192.168.5.1

source /home/nvidia/charging_robot_env/bin/activate

#!/usr/bin/env bash
# Source this file for Nova2 ROS2 / MoveIt environment.
source /opt/ros/humble/setup.bash
source /opt/charging-robot-local/nova2_ros2_ws/install/local_setup.bash
export DOBOT_TYPE=nova2
export IP_address="${CHARGING_ARM_ADDRESS:?Set the reviewed controller address}"

source /opt/charging-robot-local/charging_robot_env/bin/activate

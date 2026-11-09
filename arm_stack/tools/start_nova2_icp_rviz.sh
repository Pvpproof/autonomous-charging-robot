#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/opt/charging-robot-local/nova2_ros2_ws/install/nova2_moveit:/opt/charging-robot-local/nova2_ros2_ws/install/cr6al_moveit:/opt/charging-robot-local/nova2_ros2_ws/install/dobot_rviz:/opt/charging-robot-local/nova2_ros2_ws/install/dobot_moveit:/opt/charging-robot-local/nova2_ros2_ws/install/dobot_kinematics_plugin:/opt/charging-robot-local/nova2_ros2_ws/install/dobot_demo:/opt/charging-robot-local/nova2_ros2_ws/install/dobot_gazebo:/opt/charging-robot-local/nova2_ros2_ws/install/cra_description:/opt/ros/humble
export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH
exec rviz2 -d /opt/charging-robot-local/nova2_tools/nova2_icp.rviz

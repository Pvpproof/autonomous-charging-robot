#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/home/nvidia/nova2_ros2_ws/install/nova2_moveit:/home/nvidia/nova2_ros2_ws/install/cr6al_moveit:/home/nvidia/nova2_ros2_ws/install/dobot_rviz:/home/nvidia/nova2_ros2_ws/install/dobot_moveit:/home/nvidia/nova2_ros2_ws/install/dobot_kinematics_plugin:/home/nvidia/nova2_ros2_ws/install/dobot_demo:/home/nvidia/nova2_ros2_ws/install/dobot_gazebo:/home/nvidia/nova2_ros2_ws/install/cra_description:/opt/ros/humble
export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH
exec ros2 launch nova2_moveit demo.launch.py use_rviz:=false allow_trajectory_execution:=false

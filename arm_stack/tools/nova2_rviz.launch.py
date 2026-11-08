"""Visualization only: give RViz the same model and kinematics as MoveIt."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    mode=os.environ.get('CHARGE_ROBOT_RVIZ_VIEW','readonly')
    if mode not in ('readonly','planning'):raise ValueError('RViz view must be readonly or planning')
    path=str(Path(__file__).parent/'nova2_readonly.rviz') if mode=='readonly' else '/opt/charging-robot-local/nova2_tools/nova2_icp.rviz'
    config=MoveItConfigsBuilder('nova2_robot',package_name='nova2_moveit').to_moveit_configs()
    return LaunchDescription([Node(package='rviz2',executable='rviz2',name='nova2_rviz',output='screen',
        arguments=['-d',path],
        parameters=[config.robot_description,config.robot_description_semantic,
                    config.robot_description_kinematics,config.planning_pipelines,config.joint_limits])])

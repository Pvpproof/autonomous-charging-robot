from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    start_moveit=DeclareLaunchArgument('start_moveit',default_value='true')
    run_dock=DeclareLaunchArgument('run_dock',default_value='true')
    params=PathJoinSubstitution([FindPackageShare('charging_dock_sim'),'config','docking_params.yaml'])
    demo=IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('nova2_moveit'),'launch','demo.launch.py'])),condition=IfCondition(LaunchConfiguration('start_moveit')))
    scene=Node(package='charging_dock_sim',executable='scene_publisher',name='charging_dock_scene',output='screen',parameters=[params])
    dock=Node(package='charging_dock_sim',executable='auto_dock',name='auto_dock',output='screen',parameters=[params],condition=IfCondition(LaunchConfiguration('run_dock')))
    return LaunchDescription([start_moveit,run_dock,demo,scene,dock])

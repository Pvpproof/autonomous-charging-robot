"""Independent composition of installed vendor drivers and ROS navigation."""
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def include(package, name, **arguments):
    path = Path(get_package_share_directory(package))/'launch'/name
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments=arguments.items())


def build(context):
    mode=LaunchConfiguration('mode').perform(context)
    nodes=[]
    if mode=='devices':
        return [GroupAction([SetRemap(src='/cmd_vel',dst='/charging_control/chassis'),
                    include('turn_on_ychd_robot','turn_on_ychd_robot.launch.py')]),
                include('lslidar_driver','lsm10p_uart_launch.py'),
                ExecuteProcess(cmd=['/usr/bin/python3',str(Path(__file__).parent/'control_arbiter_node.py'),'--live'],output='screen')]
    if mode=='mapping':
        nodes.append(Node(package='slam_gmapping', executable='slam_gmapping',
                          output='screen',parameters=[{'use_sim_time':False}]))
    elif mode=='navigation':
        map_file=LaunchConfiguration('map').perform(context)
        if not Path(map_file).is_file(): raise RuntimeError('请选择已保存地图')
        params_file = os.environ.get('CHARGING_NAV2_PARAMS', '')
        if not params_file or not Path(params_file).is_file():
            raise RuntimeError('请用 CHARGING_NAV2_PARAMS 指定经许可且完成现场核验的 Nav2 配置文件；仓库不分发厂家配置')
        if Path(params_file).resolve() == (Path(__file__).parent/'examples'/'nav2.placeholder.yaml').resolve():
            raise RuntimeError('示例占位文件不能用于导航，请提供完成核验的本地配置')
        nodes.append(GroupAction([SetRemap(src='/cmd_vel',dst='/charging_control/navigation'),include('nav2_bringup','bringup_launch.py',map=map_file,
                     params_file=params_file,
                     use_sim_time='false',slam='False',autostart='true',
                     use_composition='True',use_respawn='false')]))
    else: raise ValueError('mode must be devices, mapping or navigation')
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mode',default_value='mapping'),
        DeclareLaunchArgument('map',default_value=''),OpaqueFunction(function=build)])

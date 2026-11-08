"""Keep vendor raw joint publication away from MoveIt/RSP consumers."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import GroupAction,IncludeLaunchDescription,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    vendor=Path(get_package_share_directory('dobot_bringup_v4'))/'launch/dobot_bringup_ros2.launch.py'
    return LaunchDescription([
        GroupAction([SetRemap(src='/joint_states',dst='/nova2/raw_joint_states'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(vendor)))]),
        ExecuteProcess(cmd=['/usr/bin/python3',str(Path(__file__).parent/'verified_joint_feedback.py')],output='screen')])

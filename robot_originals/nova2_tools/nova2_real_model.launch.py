from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node

URDF = Path('/home/nvidia/nova2_ros2_ws/install/dobot_rviz/share/dobot_rviz/urdf/nova2_robot.urdf')


def generate_launch_description():
    robot_description = URDF.read_text()
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='nova2_robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # Visualization-only frame for the charging-gun TCP. This does not
        # change the controller TCP and does not command robot motion.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='charging_gun_tcp_static_tf',
            output='screen',
            arguments=['--x', '0', '--y', '0', '--z', '0.123', '--roll', '0', '--pitch', '0', '--yaw', '-1.5707963267948966', '--frame-id', 'Link6', '--child-frame-id', 'charging_gun_tcp'],
        ),
        # Visualization-only camera frame from the existing hand-eye result.
        # p_Link6 = R_cam2gripper * p_camera + t_cam2gripper.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_frame_static_tf',
            output='screen',
            arguments=['--x', '0.1005272073', '--y', '-0.0236788948', '--z', '0.0294796874', '--qx', '0.0002777737', '--qy', '0.0023725564', '--qz', '0.722108856', '--qw', '0.691775320', '--frame-id', 'Link6', '--child-frame-id', 'camera_frame'],
        ),
    ])

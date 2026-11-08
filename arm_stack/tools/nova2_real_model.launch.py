"""Reconstructed model-only launch; no embedded site transforms or fake joints.

Historical source had private camera calibration. This replacement intentionally
publishes the supplied URDF only; docking frames must be configured separately.
"""
import os
from pathlib import Path
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch_ros.actions import Node

RECONSTRUCTED = True

def model_configuration(environ=None):
    environ=os.environ if environ is None else environ
    value=environ.get('CHARGING_ARM_URDF','')
    path=Path(value)
    if not value or not path.is_absolute() or not path.is_file():
        raise RuntimeError('CHARGING_ARM_URDF must name an existing reviewed absolute URDF file')
    text=path.read_text();robot=ET.fromstring(text)
    if robot.tag!='robot' or not robot.findall('link'):
        raise ValueError('Expected a resolved URDF robot with links, not a Xacro template')
    if any('xacro' in element.tag for element in robot.iter()):
        raise ValueError('Resolve Xacro with the reviewed configuration before model display')
    topic=environ.get('CHARGING_ARM_JOINT_STATES_TOPIC','/joint_states')
    if not topic.startswith('/') or any(char.isspace() for char in topic):
        raise ValueError('Joint feedback topic must be an absolute ROS topic')
    return text,topic

def generate_launch_description():
    text,topic=model_configuration()
    return LaunchDescription([Node(package='robot_state_publisher',executable='robot_state_publisher',
        name='nova2_robot_state_publisher',output='screen',
        parameters=[{'robot_description':text}],remappings=[('/joint_states',topic)])])

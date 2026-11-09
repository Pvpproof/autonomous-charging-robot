#!/usr/bin/env python3
"""Publish Dobot Nova2 real joint angles as sensor_msgs/JointState.

Read-only bridge:
  /dobot_bringup_ros2/srv/GetAngle  ->  /joint_states_robot

It does NOT enable the robot, move the robot, or call any IO/motion service.
"""
import math
import re
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dobot_msgs_v4.srv import GetAngle


def parse_robot_return(text):
    nums = [float(x) for x in re.findall(r'[-+]?\d+(?:\.\d+)?', text or '')]
    if len(nums) < 6:
        raise ValueError(f'GetAngle returned fewer than 6 numbers: {text!r}')
    return nums[:6]


class GetAngleJointStatePublisher(Node):
    def __init__(self):
        super().__init__('get_angle_joint_state_publisher')
        self.declare_parameter('service_name', '/dobot_bringup_ros2/srv/GetAngle')
        self.declare_parameter('topic_name', '/joint_states_robot')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('joint_names', ['joint1','joint2','joint3','joint4','joint5','joint6'])
        self.service_name = self.get_parameter('service_name').value
        self.topic_name = self.get_parameter('topic_name').value
        self.joint_names = list(self.get_parameter('joint_names').value)
        rate = float(self.get_parameter('rate_hz').value)
        self.cli = self.create_client(GetAngle, self.service_name)
        self.pub = self.create_publisher(JointState, self.topic_name, 10)
        self.future = None
        self.last_warn = 0
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.tick)
        self.get_logger().info(f'Read-only bridge: {self.service_name} -> {self.topic_name} @ {rate} Hz')

    def tick(self):
        if not self.cli.service_is_ready():
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_warn > 2.0:
                self.get_logger().warn(f'Waiting for service {self.service_name}')
                self.last_warn = now
            return
        if self.future is None:
            self.future = self.cli.call_async(GetAngle.Request())
            return
        if not self.future.done():
            return
        try:
            resp = self.future.result()
            self.future = None
            if getattr(resp, 'res', 0) != 0:
                self.get_logger().warn(f'GetAngle res={resp.res}, robot_return={resp.robot_return!r}')
                return
            deg = parse_robot_return(resp.robot_return)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [math.radians(v) for v in deg]
            msg.velocity = []
            msg.effort = []
            self.pub.publish(msg)
        except Exception as e:
            self.future = None
            self.get_logger().warn(f'Failed to publish joint state: {e}')


def main():
    rclpy.init()
    node = GetAngleJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

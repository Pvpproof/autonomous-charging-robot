#!/usr/bin/env python3
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint, RobotState
from moveit_msgs.srv import GetCartesianPath
from shape_msgs.msg import SolidPrimitive


def quaternion_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def make_pose(x, y, z, rpy):
    q = quaternion_from_euler(*rpy)
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    return p


def make_goal_constraints(pose, frame, link):
    c = Constraints()
    position = PositionConstraint()
    position.header.frame_id = frame
    position.link_name = link
    position.constraint_region.primitives.append(SolidPrimitive())
    position.constraint_region.primitives[0].type = SolidPrimitive.SPHERE
    position.constraint_region.primitives[0].dimensions = [0.005]
    position.constraint_region.primitive_poses.append(pose)
    position.weight = 1.0
    c.position_constraints.append(position)

    orientation = OrientationConstraint()
    orientation.header.frame_id = frame
    orientation.link_name = link
    orientation.orientation = pose.orientation
    orientation.absolute_x_axis_tolerance = 0.03
    orientation.absolute_y_axis_tolerance = 0.03
    orientation.absolute_z_axis_tolerance = 0.03
    orientation.weight = 1.0
    c.orientation_constraints.append(orientation)
    return c


class PlanOnlyDocking(Node):
    def __init__(self):
        super().__init__('plan_charging_dock')
        self.group_name = self.declare_parameter('planning_group', 'nova2_group').value
        self.ee = self.declare_parameter('end_effector_link', 'Link6').value
        self.frame = self.declare_parameter('planning_frame', 'dummy_link').value
        self.port = list(self.declare_parameter('port_xyz', [0.40, 0.0, 0.45]).value)
        self.rpy = list(self.declare_parameter('dock_rpy', [1.5708, 0.0, 1.5708]).value)
        self.pre = float(self.declare_parameter('pre_insert_distance', 0.12).value)
        self.insert = float(self.declare_parameter('insert_depth', 0.08).value)
        self.step = float(self.declare_parameter('cartesian_step', 0.005).value)
        self.min_fraction = float(self.declare_parameter('min_cartesian_fraction', 0.98).value)
        self.planning_time = float(self.declare_parameter('planning_time', 5.0).value)
        self.vel = float(self.declare_parameter('velocity_scaling', 0.20).value)
        self.acc = float(self.declare_parameter('acceleration_scaling', 0.20).value)
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.cart_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.pre_state = None

    def plan_free_space(self, pose):
        if not self.move_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action /move_action is not available.')
            return False
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.group_name
        req.goal_constraints = [make_goal_constraints(pose, self.frame, self.ee)]
        req.allowed_planning_time = self.planning_time
        req.num_planning_attempts = 5
        req.max_velocity_scaling_factor = self.vel
        req.max_acceleration_scaling_factor = self.acc
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Pre-insert planning goal was rejected.')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=20.0)
        result = result_future.result()
        if result is None:
            self.get_logger().error('No result received from MoveGroup.')
            return False
        code = result.result.planning_time
        error = result.result.error_code.val
        if error == 1:
            traj = result.result.planned_trajectory.joint_trajectory
            if traj.points:
                self.pre_state = RobotState()
                self.pre_state.joint_state.name = list(traj.joint_names)
                self.pre_state.joint_state.position = list(traj.points[-1].positions)
        self.get_logger().info(f'Pre-insert result: error_code={error} planning_time={code:.3f}s')
        return error == 1

    def plan_cartesian(self, pre_pose, dock_pose):
        if not self.cart_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('GetCartesianPath service is not available.')
            return False
        req = GetCartesianPath.Request()
        req.header.frame_id = self.frame
        req.group_name = self.group_name
        if self.pre_state is not None:
            req.start_state = self.pre_state
        req.link_name = self.ee
        req.waypoints = [pre_pose, dock_pose]
        req.max_step = self.step
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        future = self.cart_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        result = future.result()
        if result is None:
            self.get_logger().error('No result received from GetCartesianPath.')
            return False
        self.get_logger().info(f'Insertion Cartesian fraction: {result.fraction:.3f} (required >= {self.min_fraction:.3f})')
        return result.fraction >= self.min_fraction


def main():
    rclpy.init()
    node = PlanOnlyDocking()
    pre = make_pose(node.port[0] - node.pre, node.port[1], node.port[2], node.rpy)
    dock = make_pose(node.port[0] + node.insert, node.port[1], node.port[2], node.rpy)
    node.get_logger().info(f'PLAN-ONLY: group={node.group_name} ee={node.ee} frame={node.frame}')
    node.get_logger().info(f'Pre-insert target: x={pre.position.x:.3f} y={pre.position.y:.3f} z={pre.position.z:.3f}')
    node.get_logger().info(f'Dock target:       x={dock.position.x:.3f} y={dock.position.y:.3f} z={dock.position.z:.3f}')
    ok_pre = node.plan_free_space(pre)
    ok_cart = node.plan_cartesian(pre, dock) if ok_pre else False
    if ok_pre and ok_cart:
        node.get_logger().info('DOCKING PLAN PASSED: free-space approach + straight insertion verified.')
        node.get_logger().warn('This node is plan-only. It never calls execute() or sends hardware motion commands.')
        rc = 0
    else:
        node.get_logger().error('DOCKING PLAN FAILED. No execution was attempted.')
        rc = 2
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    raise SystemExit(main())

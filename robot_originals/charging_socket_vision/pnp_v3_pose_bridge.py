#!/usr/bin/env python3
"""Publish fresh read-only RGB PnP V3 poses and export the gun planner target."""
import json
import re
import time
import uuid
from pathlib import Path

import numpy as np
import rclpy
from dobot_msgs_v4.srv import GetPose
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker, MarkerArray


ROOT = Path('/home/nvidia/charging_socket_vision')
LIVE = ROOT / 'pnp_live_test_20261023_M07DN5/pnp_live_rgb_v3/live_results/latest_pose_readonly.json'
TARGET = ROOT / 'results/realtime_runs/latest_camera_socket_target.json'
REQUEST = ROOT / 'results/realtime_runs/latest_camera_plan_request.json'
HAND_EYE = Path('/home/nvidia/charging_robot_tools/calibration/hand_eye_calibration_result_ros_24mm.npz')
MAX_AGE_S = 1.0


def atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    temp.write_text(json.dumps(value, indent=2))
    temp.replace(path)


def rot_dobot(rx, ry, rz):
    x, y, z = np.deg2rad([rx, ry, rz])
    cx, sx = np.cos(x), np.sin(x)
    cy, sy = np.cos(y), np.sin(y)
    cz, sz = np.cos(z), np.sin(z)
    return np.array([[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                     [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
                     [-sy, cy*sx, cy*cx]])


class PnPV3Bridge(Node):
    def __init__(self):
        super().__init__('pnp_v3_pose_bridge')
        self.client = self.create_client(GetPose, '/dobot_bringup_ros2/srv/GetPose')
        self.pending = False
        self.Tbg = None
        self.robot_pose_time = 0.0
        self.last_signature = None
        self.last_invalid_reason = None
        self.request_id = 'pnp-v3-' + uuid.uuid4().hex
        self.plan_id = 'pnp-v3-' + str(time.time_ns())
        self.target_pub = self.create_publisher(PoseStamped, '/charging_port_target', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/icp/markers', 10)
        with np.load(HAND_EYE) as data:
            self.Tgc = np.eye(4)
            self.Tgc[:3, :3] = np.asarray(data['R_cam2gripper'], float)
            self.Tgc[:3, 3] = np.asarray(data['t_cam2gripper'], float).reshape(3)
        if np.linalg.norm(self.Tgc[:3, 3]) > 2:
            self.Tgc[:3, 3] /= 1000.
        self.create_timer(0.2, self.read_robot_pose)
        self.create_timer(0.05, self.publish_target)
        self.get_logger().info('RGB PnP V3 bridge: read-only target and RViz visualization')

    def read_robot_pose(self):
        if self.pending or not self.client.service_is_ready():
            return
        self.pending = True
        future = self.client.call_async(GetPose.Request(user=0, tool=0))
        future.add_done_callback(self.robot_pose_done)

    def robot_pose_done(self, future):
        self.pending = False
        try:
            result = future.result()
            if result is None or result.res != 0:
                return
            values = [float(v) for v in re.findall(r'[-+]?\d+(?:\.\d+)?', result.robot_return)]
            if len(values) < 6:
                return
            self.Tbg = np.eye(4)
            self.Tbg[:3, :3] = rot_dobot(*values[3:6])
            self.Tbg[:3, 3] = np.asarray(values[:3]) / 1000.
            self.robot_pose_time = time.time()
        except Exception as error:
            self.get_logger().warning(f'GetPose parse failed: {error}')

    def publish_target(self):
        try:
            if self.Tbg is None or time.time() - self.robot_pose_time > 0.5:
                self.invalidate('robot_pose_unavailable')
                return
            payload = json.loads(LIVE.read_text())
            measured = float(payload['measurement_time_host'])
            age = time.time() - measured
            matrix = np.asarray(payload.get('T_camera_from_socket'), dtype=float)
            if (payload.get('pose_valid') is not True or not 0 <= age <= MAX_AGE_S
                    or matrix.shape != (4, 4) or not np.isfinite(matrix).all()
                    or not np.allclose(matrix[3], [0, 0, 0, 1])
                    or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-3)
                    or abs(np.linalg.det(matrix[:3, :3]) - 1) > 1e-3):
                self.invalidate('visual_pose_invalid:' + str(payload.get('reason', 'unknown')))
                return
            Tbs = self.Tbg @ self.Tgc @ matrix
            signature = (payload.get('frame_id'), measured)
            if signature != self.last_signature:
                request = {'request_id': self.request_id, 'requested_at': measured,
                           'updated_at': time.time(), 'status': 'complete',
                           'stage': 'camera', 'plan_id': self.plan_id,
                           'planning_ready': False, 'source': 'rgb_pnp_v3',
                           'measurement_time': measured}
                target = {'schema_version': 1, 'stage': 'camera_socket_target',
                          'request_id': self.request_id, 'plan_id': self.plan_id,
                          'camera_plan_at': measured, 'measurement_time': measured,
                          'pose_valid': True, 'pose_source': 'rgb_pnp_v3',
                          'T_base_from_socket': Tbs.tolist()}
                atomic_json(TARGET, target)
                atomic_json(REQUEST, request)
                self.last_signature = signature

            message = PoseStamped()
            message.header = Header()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'base_link'
            message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, Tbs[:3, 3])
            q = Rotation.from_matrix(Tbs[:3, :3]).as_quat()
            message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = map(float, q)
            self.target_pub.publish(message)
            self.last_invalid_reason = None
            array = MarkerArray()
            for index, (axis, color) in enumerate(zip(np.eye(3), [(1., 0., 0., 1.), (0., 1., 0., 1.), (0., 0., 1., 1.)])):
                marker = Marker()
                marker.header = message.header
                marker.ns, marker.id, marker.type, marker.action = 'pnp_v3_socket_axes', index, Marker.ARROW, Marker.ADD
                marker.lifetime=Duration(sec=0,nanosec=500000000)
                marker.scale.x, marker.scale.y, marker.scale.z = .035, .0015, .0015
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
                from geometry_msgs.msg import Point
                start, end = Point(), Point()
                start.x, start.y, start.z = map(float, Tbs[:3, 3])
                tip = Tbs[:3, 3] + Tbs[:3, :3] @ axis * .035
                end.x, end.y, end.z = map(float, tip)
                marker.points = [start, end]
                array.markers.append(marker)
            self.marker_pub.publish(array)
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.invalidate('pose_file_unavailable_or_malformed')
            self.get_logger().warning(f'V3 target unavailable: {error}', throttle_duration_sec=3.0)

    def invalidate(self, reason):
        if self.last_invalid_reason is None:
            array=MarkerArray()
            for index in range(3):
                marker=Marker(); marker.header.frame_id='base_link'
                marker.ns,marker.id,marker.action='pnp_v3_socket_axes',index,Marker.DELETE
                array.markers.append(marker)
            self.marker_pub.publish(array)
        if reason == self.last_invalid_reason:
            return
        request = {'request_id': self.request_id, 'requested_at': time.time(),
                   'updated_at': time.time(), 'status': 'invalid', 'stage': 'camera',
                   'plan_id': self.plan_id, 'planning_ready': False,
                   'source': 'rgb_pnp_v3', 'error': reason}
        target = {'schema_version': 1, 'stage': 'camera_socket_target',
                  'request_id': self.request_id, 'plan_id': self.plan_id,
                  'camera_plan_at': 0, 'measurement_time': None,
                  'pose_valid': False, 'pose_source': 'rgb_pnp_v3',
                  'T_base_from_socket': None}
        atomic_json(TARGET, target)
        atomic_json(REQUEST, request)
        self.last_invalid_reason = reason


def main():
    rclpy.init()
    node = PnPV3Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

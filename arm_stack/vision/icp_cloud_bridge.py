#!/usr/bin/env python3
import json
import time
from pathlib import Path
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
from dobot_msgs_v4.srv import GetPose
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation


ROOT = Path('/opt/charging-robot-local/charging_socket_vision')
POSE = ROOT / 'results/v3_diagnostics/latest_pose_readonly.json'
CLOUD = ROOT / 'results/v3_diagnostics/latest_target_camera.ply'
RGBD_CLOUD = ROOT / 'results/v3_diagnostics/latest_rgbd_camera.ply'
HAND_EYE = Path('/opt/charging-robot-local/charging_robot_tools/calibration/hand_eye_calibration_result_ros_24mm.npz')


def rot_dobot(rx, ry, rz):
    rx, ry, rz = np.deg2rad([rx, ry, rz])
    cx, sx = np.cos(rx), np.sin(rx); cy, sy = np.cos(ry), np.sin(ry); cz, sz = np.cos(rz), np.sin(rz)
    return np.array([[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                     [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
                     [-sy, cy*sx, cy*cx]], dtype=float)

def tf_msg(parent, child, T):
    m = TransformStamped()
    m.header.stamp = rclpy.time.Time().to_msg()
    m.header.frame_id, m.child_frame_id = parent, child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, T[:3, 3])
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = map(float, q)
    return m

def transform_from_tf(t):
    q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
    T = np.eye(4); T[:3, :3] = Rotation.from_quat(q).as_matrix()
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T

def cloud_msg(points, frame, stamp, colors=None):
    if colors is None:
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        rows = np.asarray(points, dtype=np.float32).tolist()
    else:
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                  PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]
        p = np.asarray(points, dtype=np.float32)
        c = np.clip(np.asarray(colors) * 255.0, 0, 255).astype(np.uint32)
        packed = (c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]
        rows = [list(x[:3]) + [int(v)] for x, v in zip(p.tolist(), packed)]
    header = Header(); header.stamp = stamp; header.frame_id = frame
    return point_cloud2.create_cloud(header, fields, rows)

def pose_msg(T, frame, stamp):
    p = PoseStamped(); p.header.frame_id, p.header.stamp = frame, stamp
    p.pose.position.x, p.pose.position.y, p.pose.position.z = map(float, T[:3, 3])
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = map(float, q)
    return p

class IcpCloudBridge(Node):
    def __init__(self):
        super().__init__('icp_cloud_bridge')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('gripper_frame', 'Link6')
        self.declare_parameter('camera_frame', 'realsense_color_optical_frame')
        self.declare_parameter('target_topic', '/charging_port_target')
        self.declare_parameter('publish_hz', 5.0)
        self.declare_parameter('use_online_pose', True)
        self.base = self.get_parameter('base_frame').value
        self.gripper = self.get_parameter('gripper_frame').value
        self.visual_fix = np.eye(4)
        self.camera = self.get_parameter('camera_frame').value
        self.last_cloud_mtime = self.last_pose_mtime = 0.0
        self.points_cam = self.colors_cam = self.rgbd_points_cam = self.rgbd_colors_cam = self.Tcs = None
        self.Tbc_online = None
        self.robot_pose_time = 0.0
        self.visual_measurement_time = None
        self.markers_visible = True
        self.pose_client = self.create_client(GetPose, '/dobot_bringup_ros2/srv/GetPose')
        self.pose_pending = False
        self.cloud_pub = self.create_publisher(PointCloud2, '/icp/pointcloud_base', 2)
        self.rgbd_pub = self.create_publisher(PointCloud2, '/icp/rgbd_pointcloud_base', 2)
        self.target_pub = self.create_publisher(PoseStamped, self.get_parameter('target_topic').value, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/icp/markers', 10)
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        # In online-pose mode, do not publish a competing Link6->camera TF.
        self.static = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / float(self.get_parameter('publish_hz').value), self.tick)
        self.pose_timer = self.create_timer(0.2, self.request_online_pose)
        self.get_logger().info('ICP cloud bridge: read-only visualization; no robot command path')

    def request_online_pose(self):
        if not self.get_parameter('use_online_pose').value or self.pose_pending:
            return
        if not self.pose_client.wait_for_service(timeout_sec=0.05):
            self.get_logger().warn('GetPose service not ready yet...', throttle_duration_sec=3.0)
            return
        self.pose_pending = True
        self.get_logger().info('Requesting live GetPose', throttle_duration_sec=3.0)
        future = self.pose_client.call_async(GetPose.Request(user=0, tool=0))
        future.add_done_callback(self.online_pose_done)

    def online_pose_done(self, future):
        self.pose_pending = False
        try:
            if future.exception() is not None:
                self.get_logger().error(f'GetPose future exception: {future.exception()}')
                return
            response = future.result()
            self.get_logger().info(f'GetPose response received: res={getattr(response, "res", None)}')
            if response is None or response.res != 0:
                self.get_logger().warn(f'GetPose failed: {response}', throttle_duration_sec=3.0)
                return
            import re
            values = [float(x) for x in re.findall(r'[-+]?\d+(?:\.\d+)?', response.robot_return)]
            if len(values) < 6:
                return
            Tbg = np.eye(4)
            Tbg[:3, :3] = rot_dobot(*values[3:6])
            Tbg[:3, 3] = np.asarray(values[:3], dtype=float) / 1000.0
            with np.load(HAND_EYE) as d:
                Tgc = np.eye(4)
                Tgc[:3, :3] = np.asarray(d['R_cam2gripper'], float)
                Tgc[:3, 3] = np.asarray(d['t_cam2gripper'], float).reshape(3)
            if np.linalg.norm(Tgc[:3, 3]) > 2:
                Tgc[:3, 3] /= 1000.0
            self.Tbc_online = Tbg @ Tgc
            self.robot_pose_time = time.time()
        except Exception as e:
            self.get_logger().warn(f'GetPose parse failed: {e}', throttle_duration_sec=3.0)

    def publish_handeye_tf(self):
        try:
            with np.load(HAND_EYE) as d:
                R = np.asarray(d['R_cam2gripper'], float)
                t = np.asarray(d['t_cam2gripper'], float).reshape(3)
            if np.linalg.norm(t) > 2: t /= 1000.0
            # Calibration file stores T_gripper_from_camera (cam2gripper).
            # TF needs T_camera_from_gripper for the edge Link6 -> camera.
            T_gripper_camera = np.eye(4); T_gripper_camera[:3, :3], T_gripper_camera[:3, 3] = R, t
            T_camera_gripper = np.linalg.inv(T_gripper_camera)
            self.static.sendTransform(tf_msg(self.gripper, self.camera, T_camera_gripper))
            self.get_logger().info(f'Published corrected hand-eye TF {self.gripper}->{self.camera}, t={T_camera_gripper[:3,3].tolist()}')
        except Exception as e: self.get_logger().error(f'hand-eye load failed: {e}')

    def load(self):
        try:
            if CLOUD.exists() and CLOUD.stat().st_mtime != self.last_cloud_mtime:
                q = o3d.io.read_point_cloud(str(CLOUD)); p = np.asarray(q.points, dtype=float)
                good = np.isfinite(p).all(axis=1); self.points_cam = p[good]
                c = np.asarray(q.colors, dtype=float)
                self.colors_cam = c[good] if len(c) == len(p) else None
                self.last_cloud_mtime = CLOUD.stat().st_mtime
                self.get_logger().info(f'Loaded ICP cloud: {len(self.points_cam)} points')
            if RGBD_CLOUD.exists() and RGBD_CLOUD.stat().st_mtime != getattr(self, 'last_rgbd_mtime', 0.0):
                q = o3d.io.read_point_cloud(str(RGBD_CLOUD)); p = np.asarray(q.points, dtype=float)
                good = np.isfinite(p).all(axis=1); self.rgbd_points_cam = p[good]
                c = np.asarray(q.colors, dtype=float)
                self.rgbd_colors_cam = c[good] if len(c) == len(p) else None
                self.last_rgbd_mtime = RGBD_CLOUD.stat().st_mtime
                self.get_logger().info(f'Loaded RGB-D cloud: {len(self.rgbd_points_cam)} points')
            if POSE.exists() and POSE.stat().st_mtime != self.last_pose_mtime:
                payload = json.loads(POSE.read_text())
                valid = (payload.get('pose_valid') is True and payload.get('pose_fresh') is True
                         and payload.get('quality_level')=='full_geometry')
                self.visual_measurement_time = payload.get('measurement_time')
                matrix = payload.get('T_camera_from_socket_raw')
                self.Tcs = np.asarray(matrix, dtype=float) if valid and matrix is not None else None
                self.last_pose_mtime = POSE.stat().st_mtime
                self.get_logger().info(
                    f"Loaded ICP pose: valid={valid} status={payload.get('status')} "
                    f"age_s={payload.get('measurement_age_s')}"
                )
            if not POSE.exists(): self.Tcs = None
            if not CLOUD.exists(): self.points_cam = None
        except Exception as e:
            self.Tcs = None
            self.get_logger().warn(f'input read failed: {e}')

    def tick(self):
        self.load(); stamp = self.get_clock().now().to_msg()
        now=time.time()
        visual_fresh=(self.Tcs is not None and isinstance(self.visual_measurement_time,(int,float))
                      and not isinstance(self.visual_measurement_time,bool)
                      and 0<=now-self.visual_measurement_time<=1.0)
        if not visual_fresh: self.clear_markers(stamp)
        try:
            if self.get_parameter('use_online_pose').value:
                if self.Tbc_online is None or not 0<=now-self.robot_pose_time<=0.5:
                    self.clear_markers(stamp)
                    self.get_logger().warn('Waiting for live GetPose...', throttle_duration_sec=3.0)
                    return
                Tbc = self.Tbc_online
            else:
                tr = self.tf.lookup_transform(self.base, self.camera, rclpy.time.Time())
                Tbc = transform_from_tf(tr)
            # Publish the pose first so a PointCloud2 packing error cannot suppress the target.
            if visual_fresh:
                Tbs = Tbc @ self.Tcs
                self.target_pub.publish(pose_msg(Tbs, self.base, stamp))
                self.publish_markers(Tbs, stamp)
                self.get_logger().info('Published live charging target', throttle_duration_sec=3.0)
            if self.points_cam is None or not CLOUD.exists() or not 0<=now-CLOUD.stat().st_mtime<=1.0: return
            p = (Tbc[:3, :3] @ self.points_cam.T + Tbc[:3, 3:4]).T
            step = max(1, int(len(p) / 30000))
            colors = self.colors_cam[::step] if self.colors_cam is not None else None
            self.cloud_pub.publish(cloud_msg(p[::step], self.base, stamp, colors))
            if self.rgbd_points_cam is not None:
                rp = (Tbc[:3, :3] @ self.rgbd_points_cam.T + Tbc[:3, 3:4]).T
                rs = max(1, int(len(rp) / 50000)); rc = self.rgbd_colors_cam[::rs] if self.rgbd_colors_cam is not None else None
                self.rgbd_pub.publish(cloud_msg(rp[::rs], self.base, stamp, rc))
        except Exception as e:
            self.clear_markers(stamp)
            self.get_logger().error(f'publish exception: {type(e).__name__}: {e}', throttle_duration_sec=3.0)

    def clear_markers(self, stamp):
        if not self.markers_visible: return
        arr=MarkerArray()
        for i in range(3):
            m=Marker(); m.header.frame_id=self.base; m.header.stamp=stamp
            m.ns,m.id,m.action='icp_socket_axes',i,Marker.DELETE
            arr.markers.append(m)
        self.marker_pub.publish(arr); self.markers_visible=False

    def publish_markers(self, T, stamp):
        arr = MarkerArray()
        for i, (axis, col) in enumerate(zip(np.eye(3), [(1.,0.,0.,1.), (0.,1.,0.,1.), (0.,0.,1.,1.)])):
            m = Marker(); m.header.frame_id, m.header.stamp = self.base, stamp
            m.ns, m.id, m.type, m.action = 'icp_socket_axes', i, Marker.ARROW, Marker.ADD
            m.lifetime=Duration(sec=0,nanosec=500000000)
            m.scale.x, m.scale.y, m.scale.z = .035, .0015, .0015
            m.color.r, m.color.g, m.color.b, m.color.a = col
            p0, p1 = Point(), Point(); p0.x, p0.y, p0.z = map(float, T[:3,3])
            v = T[:3,:3] @ axis * .035; p1.x, p1.y, p1.z = map(float, T[:3,3] + v)
            m.points = [p0, p1]; arr.markers.append(m)
        self.marker_pub.publish(arr)
        self.markers_visible=True

def main():
    rclpy.init(); node = IcpCloudBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()

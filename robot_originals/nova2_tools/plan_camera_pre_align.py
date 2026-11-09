#!/usr/bin/env python3
import json, re, sys
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, DisplayTrajectory, RobotState
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from dobot_msgs_v4.srv import GetPose
from scipy.spatial.transform import Rotation

POSE_JSON=Path('/home/nvidia/charging_socket_vision/results/realtime_runs/latest_pose_readonly.json')
HAND_EYE=Path('/home/nvidia/charging_robot_tools/calibration/hand_eye_calibration_result_ros_24mm.npz')
DIST=0.150
BASE='base_link'; GROUP='nova2_group'; EE='Link6'

def rot_dobot(rx,ry,rz):
    rx,ry,rz=np.deg2rad([rx,ry,rz]); cx,sx=np.cos(rx),np.sin(rx); cy,sy=np.cos(ry),np.sin(ry); cz,sz=np.cos(rz),np.sin(rz)
    return np.array([[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx],[sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx],[-sy,cy*sx,cy*cx]])

def pose_constraint(T):
    p=Pose(); p.position.x,p.position.y,p.position.z=map(float,T[:3,3]); q=Rotation.from_matrix(T[:3,:3]).as_quat(); p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,q)
    c=Constraints(); pc=PositionConstraint(); pc.header.frame_id=BASE; pc.link_name=EE; s=SolidPrimitive(); s.type=SolidPrimitive.SPHERE; s.dimensions=[0.008]; pc.constraint_region.primitives=[s]; pc.constraint_region.primitive_poses=[p]; pc.weight=1.0; c.position_constraints=[pc]
    oc=OrientationConstraint(); oc.header.frame_id=BASE; oc.link_name=EE; oc.orientation=p.orientation; oc.absolute_x_axis_tolerance=.04; oc.absolute_y_axis_tolerance=.04; oc.absolute_z_axis_tolerance=.04; oc.weight=1.0; c.orientation_constraints=[oc]; return c,T

class Planner(Node):
    def __init__(self):
        super().__init__('camera_pre_align_plan_only')
        self.getpose=self.create_client(GetPose,'/dobot_bringup_ros2/srv/GetPose')
        self.move=ActionClient(self,MoveGroup,'/move_action')
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos=QoSProfile(depth=1); qos.reliability=ReliabilityPolicy.RELIABLE; qos.durability=DurabilityPolicy.TRANSIENT_LOCAL
        self.display_pub=self.create_publisher(DisplayTrajectory,'/display_planned_path',qos)
    def get_target(self):
        if not self.getpose.wait_for_service(8): raise RuntimeError('GetPose unavailable')
        f=self.getpose.call_async(GetPose.Request(user=0,tool=0)); rclpy.spin_until_future_complete(self,f,timeout_sec=8); res=f.result()
        if res is None or res.res!=0: raise RuntimeError(f'GetPose failed: {res}')
        vals=[float(x) for x in re.findall(r'[-+]?\d+(?:\.\d+)?',res.robot_return)]
        if len(vals)<6: raise RuntimeError(res.robot_return)
        Tbg=np.eye(4); Tbg[:3,:3]=rot_dobot(*vals[3:6]); Tbg[:3,3]=np.asarray(vals[:3])/1000.0
        with np.load(HAND_EYE) as d:
            Tgc=np.eye(4); Tgc[:3,:3]=np.asarray(d['R_cam2gripper'],float); Tgc[:3,3]=np.asarray(d['t_cam2gripper'],float).reshape(3)
        if np.linalg.norm(Tgc[:3,3])>2:Tgc[:3,3]/=1000.0
        d=json.loads(POSE_JSON.read_text()); Tcs=np.asarray(d['T_camera_from_socket_raw'],float); Tbs=Tbg@Tgc@Tcs
        # Socket +Z points outward. Place camera 150 mm outward; optical +Z points inward.
        Rs=Tbs[:3,:3]; o=Tbs[:3,3]; cam_pos=o+Rs[:,2]*DIST
        Rc=np.column_stack((Rs[:,0],-Rs[:,1],-Rs[:,2]))
        Tbc=np.eye(4); Tbc[:3,:3]=Rc; Tbc[:3,3]=cam_pos
        Tgb=Tgc.copy(); Tgb_inv=np.linalg.inv(Tgc); Tgoal=Tbc@Tgb_inv
        self.get_logger().info(f'ICP status={d.get("status")} fitness={d.get("fitness")} rmse_m={d.get("rmse_m")}')
        self.get_logger().info(f'camera target xyz={cam_pos.tolist()} distance={DIST}m')
        self.get_logger().info(f'Link6 target xyz={Tgoal[:3,3].tolist()} rpy_deg={Rotation.from_matrix(Tgoal[:3,:3]).as_euler("xyz",degrees=True).tolist()}')
        return Tgoal
    def run(self):
        T=self.get_target()
        if not self.move.wait_for_server(10): raise RuntimeError('MoveGroup /move_action unavailable')
        goal=MoveGroup.Goal(); goal.request.group_name=GROUP; c,_=pose_constraint(T); goal.request.goal_constraints=[c]; goal.request.allowed_planning_time=10.0; goal.request.num_planning_attempts=10; goal.request.max_velocity_scaling_factor=.10; goal.request.max_acceleration_scaling_factor=.10; goal.planning_options.plan_only=True; goal.planning_options.look_around=False; goal.planning_options.replan=False
        f=self.move.send_goal_async(goal); rclpy.spin_until_future_complete(self,f,timeout_sec=15); h=f.result()
        if h is None or not h.accepted: raise RuntimeError('planning goal rejected')
        rf=h.get_result_async(); rclpy.spin_until_future_complete(self,rf,timeout_sec=30); result=rf.result().result
        self.get_logger().info(f'PLAN_RESULT error_code={result.error_code.val} planning_time={result.planning_time:.3f}s')
        jt=result.planned_trajectory.joint_trajectory
        self.get_logger().info(f'planned points={len(jt.points)} joints={list(jt.joint_names)}')
        display=DisplayTrajectory(); display.model_id='nova2_robot'; display.trajectory_start=RobotState(); display.trajectory=[result.planned_trajectory]; self.display_pub.publish(display)
        self.get_logger().info('Published plan to /display_planned_path; keep this node alive for RViz')
        self.create_timer(1.0, lambda: self.display_pub.publish(display))
        if jt.points:
            self.get_logger().info(f'first={list(jt.points[0].positions)}')
            self.get_logger().info(f'last={list(jt.points[-1].positions)}')
        return result.error_code.val==1

def main():
    rclpy.init(); n=Planner()
    try:
        ok=n.run(); rc=0 if ok else 2
        if ok:
            n.get_logger().info('PLAN-ONLY complete; keeping DisplayTrajectory alive. No execution requested.')
            rclpy.spin(n)
    except KeyboardInterrupt:
        rc=0
    except Exception as e:
        n.get_logger().error(str(e)); rc=3
    n.destroy_node(); rclpy.shutdown(); return rc
if __name__=='__main__': sys.exit(main())

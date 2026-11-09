#!/usr/bin/env python3
"""Plan-only charging-gun prepose for the Nova2.

No robot motion is sent. The target is computed from the latest completed
cam-plan socket pose. The requested theoretical gun TCP is represented
relative to Link6: (0,0,123 mm, 0,0,-90 deg).

At the prepose:
  gun +Z = socket -Z (insertion direction)
  gun +Y = socket +Y (socket top / gun flat top up)
  gun +X = socket -X (right-handed frame)
"""
import json, math, os, sys, time, uuid
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, DisplayTrajectory, RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from gun_pose_validation import JOINT_TOLERANCE_RAD, tcp_from_link6, require, evaluate, load_compensation, compensated_target
from stable_pose_selection import atomic_json
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation

SOCKET_TARGET = Path('/home/nvidia/charging_socket_vision/results/realtime_runs/frozen_pnp_v3_socket_target.json')
CAMERA_REQUEST = Path('/home/nvidia/charging_socket_vision/results/realtime_runs/frozen_pnp_v3_request.json')
ICP_SELECTION = Path('/home/nvidia/charging_socket_vision/results/realtime_runs/latest_camera_pose_selection.json')
PENDING_TRAJECTORY = Path('/home/nvidia/charging_socket_vision/results/realtime_runs/pending_trajectory.json')
REPORT = Path('/home/nvidia/charging_socket_vision/results/realtime_runs/latest_gun_prepose_plan_only.json')
GUN_TRAJ_FILE = Path('/home/nvidia/charging_socket_vision/results/realtime_runs/pending_gun_trajectory.json')
JOINTS = [f'joint{i}' for i in range(1,7)]
GROUP = 'nova2_group'
EE = 'Link6'
BASE = 'base_link'
PREPOSE_DISTANCE_M = 0.050
INSERTION_DEPTH_M = max(0.0, float(os.environ.get("INSERTION_DEPTH_MM", "0")) / 1000.0)
TCP_Z_M = 0.123
TCP_RZ = math.radians(-90.0)


def constraints(vals):
    c=Constraints()
    for name,value in zip(JOINTS, vals):
        j=JointConstraint(); j.joint_name=name; j.position=float(value)
        j.tolerance_above=JOINT_TOLERANCE_RAD; j.tolerance_below=JOINT_TOLERANCE_RAD; j.weight=1.0
        c.joint_constraints.append(j)
    return c


class Planner(Node):
    def __init__(self):
        super().__init__('charging_gun_prepose_plan_only')
        self.report={'planning_ready':False,'planning_status':'starting','motion_commands_sent':False}
        self.fk_cli=self.create_client(GetPositionFK,'/compute_fk')
        self.ik_cli=self.create_client(GetPositionIK,'/compute_ik')
        self.move=ActionClient(self,MoveGroup,'/move_action')
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos=QoSProfile(depth=1); qos.reliability=ReliabilityPolicy.RELIABLE; qos.durability=DurabilityPolicy.TRANSIENT_LOCAL
        self.pub=self.create_publisher(DisplayTrajectory,'/display_planned_path',qos)
        self.tf_broadcaster=TransformBroadcaster(self)
        self.planned_tf=None
        self.joints=None
        from sensor_msgs.msg import JointState
        self.create_subscription(JointState,'/joint_states',self.joint_cb,10)

    def joint_cb(self,msg):
        d=dict(zip(msg.name,msg.position))
        if all(n in d for n in JOINTS): self.joints=np.array([d[n] for n in JOINTS])

    def call(self,cli,req,timeout=10):
        if not cli.wait_for_service(timeout_sec=timeout): raise RuntimeError(f'服务不可用: {cli.srv_name}')
        f=cli.call_async(req); rclpy.spin_until_future_complete(self,f,timeout_sec=timeout)
        if f.result() is None: raise RuntimeError(f'服务无响应: {cli.srv_name}')
        return f.result()

    def calculate(self):
        if INSERTION_DEPTH_M>0:raise RuntimeError('禁止以RRT关节终点规划接触插入；请使用charge_robot insert-plan')
        try:
            target=json.loads(SOCKET_TARGET.read_text())
            request=json.loads(CAMERA_REQUEST.read_text())
        except (OSError,ValueError) as error:
            raise RuntimeError(f'cam-plan插座位姿/请求记录不可读: {error}')
        if (target.get('stage')!='camera_socket_target' or target.get('schema_version')!=1
                or request.get('status')!='complete'
                or target.get('request_id')!=request.get('request_id')
                or target.get('plan_id')!=request.get('plan_id')):
            raise RuntimeError('冻结的RGB PnP V3插座位姿记录无效；请重新运行PnP-V3并冻结新测量')
        if target.get('pose_source') != 'rgb_pnp_v3' or target.get('pose_valid') is not True:
            raise RuntimeError('gun-plan仅接受有效的RGB PnP V3纯图像位姿')
        Tbs=np.asarray(target.get('T_base_from_socket'),float)
        if (Tbs.shape!=(4,4) or not np.isfinite(Tbs).all()
                or not np.allclose(Tbs[3],[0,0,0,1])
                or not np.allclose(Tbs[:3,:3].T@Tbs[:3,:3],np.eye(3),atol=1e-3)
                or abs(np.linalg.det(Tbs[:3,:3])-1)>1e-3):
            raise RuntimeError('cam-plan插座基座位姿无效')
        camera_plan_age=time.time()-float(target.get('measurement_time',0))
        frozen_age=time.time()-float(target.get('frozen_at',0))
        from gun_target_policy import fuse
        cycle=json.loads((REPORT.parent/'workflow_cycle.json').read_text())
        Tbs,fusion=fuse(target,cycle)
        # Frozen base-frame target remains usable while the scene is unchanged.
        # Live measurement freshness is checked when the snapshot is acquired.
        # Keep planning anchored to the frozen RGB PnP V3 socket pose; current joints
        # below are used only as MoveIt start state.
        socket=Tbs[:3,3]
        Rs=Tbs[:3,:3]
        # Same socket axis, 50mm outside socket; gun flat top follows socket +Y.
        compensation=load_compensation()
        compensated=compensated_target(Tbs,compensation)
        tcp_pos=compensated[:3,3]*1000.
        print(f"[横向补偿] 启用={compensation['enabled']}；沿枪头朝充电口看向左 {compensation['left_mm']:.1f} mm；轴向距离50 mm",flush=True)
        Rtcp=compensated[:3,:3]
        if INSERTION_DEPTH_M > 0:
            tcp_pos = tcp_pos + Rtcp[:,2] * (INSERTION_DEPTH_M * 1000.0)
            print(f"[插入目标] 沿枪头+Z推进 {INSERTION_DEPTH_M*1000:.1f} mm（一次性规划）",flush=True)
        print(f"[方向补偿] 绕枪头TCP向右 {compensation.get('right_yaw_deg',0):.3f} 度",flush=True)
        self.planning_ready=False
        # Controller TCP: Rtcp = Rlink6 * Rz(-90), p_tcp=p_link6+Rlink6*[0,0,123].
        Rrel=np.array([[0.,1.,0.],[-1.,0.,0.],[0.,0.,1.]])
        Rlink=Rtcp@Rrel.T
        link6=tcp_pos - Rlink@np.array([0.,0.,TCP_Z_M*1000.])
        out={'read_only':True,'motion_commands_sent':False,
             'camera_plan_id':target.get('plan_id'),'camera_request_id':target.get('request_id'),
             'camera_plan_age_s':camera_plan_age,'socket_pose_source':target.get('pose_source'),
             'pose_source':('fused_icp_v3' if fusion.get('enabled') else target.get('pose_source')),
             'fusion':fusion,'cycle_id':target['cycle_id'],
             'target_stages':{'pnp_socket_base':target['T_base_from_socket'],
                              'fused_socket_base':Tbs.tolist(),'compensated_tcp_base':compensated.tolist()},
             'angular_compensation_axis':'tcp_local_y',
             'lateral_compensation':compensation,
             'lateral_compensation_base_mm':((compensated[:3,3]-(socket+Rs[:,2]*PREPOSE_DISTANCE_M))*1000.).tolist(),
             'pose_selection':target.get('pose_selection'),
             'planning_ready':self.planning_ready,
             'prepose_distance_mm':PREPOSE_DISTANCE_M*1000,'insertion_depth_mm':INSERTION_DEPTH_M*1000,
             'tcp_definition_mm_deg':[0,0,123,0,0,-90],
             'socket_origin_base_mm':(socket*1000.0).tolist(),'tcp_prepose_base_mm':tcp_pos.tolist(),
             'link6_target_base_mm':link6.tolist(),
             'gun_plus_x_base':Rtcp[:,0].tolist(),'gun_plus_y_base':Rtcp[:,1].tolist(),'gun_plus_z_base':Rtcp[:,2].tolist(),
             'socket_minus_z_base':(-Rs[:,2]).tolist(),
             'z_alignment_dot':float(np.dot(Rtcp[:,2],-Rs[:,2])),
             'gun_y_up_definition':'gun +Y = socket +Y; flat top upward',
             'distance_definition':('50 mm axial standoff plus %.1f mm insertion' % (INSERTION_DEPTH_M*1000)),
             'T_base_from_socket':Tbs.tolist()}
        self.report=out
        atomic_json(REPORT,out)
        return link6, Rlink, out, np.asarray(tcp_pos,float), np.asarray(Rtcp,float)

    def publish_planned_tcp(self):
        if self.planned_tf is not None:
            self.planned_tf.header.stamp=self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.planned_tf)

    def check_endpoint(self,names,positions,target):
        if len(names)!=6 or set(names)!=set(JOINTS) or len(positions)!=6 or not np.isfinite(positions).all():
            raise RuntimeError('枪头终点关节数据无效')
        request=GetPositionFK.Request();request.header.frame_id=BASE;request.fk_link_names=[EE]
        request.robot_state.joint_state.name=list(names)
        request.robot_state.joint_state.position=list(map(float,positions))
        response=self.call(self.fk_cli,request,timeout=8)
        if response.error_code.val!=1 or len(response.pose_stamped)!=1:
            raise RuntimeError('MoveIt FK不可用或失败，未验收枪头终点')
        stamped=response.pose_stamped[0]
        if stamped.header.frame_id.lstrip('/')!=BASE:
            raise RuntimeError('FK返回坐标系不匹配')
        pose=stamped.pose;T=np.eye(4)
        T[:3,3]=[pose.position.x,pose.position.y,pose.position.z]
        T[:3,:3]=Rotation.from_quat([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]).as_matrix()
        actual=tcp_from_link6(T);evidence=evaluate(actual,target)
        evidence.update(joint_names=list(names),joint_positions=list(map(float,positions)),
                        fk_source='/compute_fk',checked_at=time.time(),joint_tolerance_rad=JOINT_TOLERANCE_RAD)
        self.report['endpoint_validation']=evidence;atomic_json(REPORT,self.report)
        require(actual,target)
        print(f"[规划预测末端验收通过] 位置误差{evidence['position_error_mm']:.3f}mm，方向误差{evidence['rotation_error_deg']:.3f}度",flush=True)
        return evidence

    def plan(self):
        PENDING_TRAJECTORY.unlink(missing_ok=True)
        GUN_TRAJ_FILE.unlink(missing_ok=True)
        link6, Rlink, out, tcp_pos, Rtcp=self.calculate()
        for _ in range(60):
            rclpy.spin_once(self,timeout_sec=.1)
            if self.joints is not None: break
        if self.joints is None: raise RuntimeError('未收到有效 /joint_states')
        seed=self.joints.copy()
        qtcp=Rotation.from_matrix(Rtcp).as_quat()
        tf=TransformStamped(); tf.header.frame_id=BASE; tf.child_frame_id='planned_gun_tcp'
        tf.transform.translation.x=float(tcp_pos[0]/1000.0); tf.transform.translation.y=float(tcp_pos[1]/1000.0); tf.transform.translation.z=float(tcp_pos[2]/1000.0)
        tf.transform.rotation.x=float(qtcp[0]); tf.transform.rotation.y=float(qtcp[1]); tf.transform.rotation.z=float(qtcp[2]); tf.transform.rotation.w=float(qtcp[3])
        self.planned_tf=tf
        self.publish_planned_tcp()
        pose=PoseStamped(); pose.header.frame_id=BASE
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=(link6/1000.).tolist()
        q=Rotation.from_matrix(Rlink).as_quat()
        pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=q.tolist()
        if not self.ik_cli.wait_for_service(timeout_sec=8): raise RuntimeError('compute_ik不可用')
        req=GetPositionIK.Request(); req.ik_request.group_name=GROUP; req.ik_request.ik_link_name=EE
        req.ik_request.pose_stamped=pose; req.ik_request.robot_state.joint_state.name=JOINTS
        req.ik_request.robot_state.joint_state.position=seed.tolist(); req.ik_request.timeout.sec=2; req.ik_request.avoid_collisions=True
        ik=self.call(self.ik_cli,req,8)
        if ik.error_code.val!=1: raise RuntimeError(f'IK失败 code={ik.error_code.val}')
        d=dict(zip(ik.solution.joint_state.name,ik.solution.joint_state.position)); goal=np.array([d[n] for n in JOINTS])
        if not self.move.wait_for_server(timeout_sec=10): raise RuntimeError('MoveGroup不可用')
        g=MoveGroup.Goal(); g.request.group_name=GROUP; g.request.start_state.joint_state.name=JOINTS
        g.request.start_state.joint_state.position=seed.tolist(); g.request.goal_constraints=[constraints(goal)]
        g.request.allowed_planning_time=10.; g.request.num_planning_attempts=6
        g.request.max_velocity_scaling_factor=.15; g.request.max_acceleration_scaling_factor=.10
        g.request.pipeline_id='ompl'; g.request.planner_id='RRTConnect'; g.planning_options.plan_only=True
        f=self.move.send_goal_async(g); rclpy.spin_until_future_complete(self,f,timeout_sec=15); h=f.result()
        if h is None or not h.accepted: raise RuntimeError('规划目标被拒绝')
        rf=h.get_result_async(); rclpy.spin_until_future_complete(self,rf,timeout_sec=35)
        if rf.result() is None:raise RuntimeError('规划结果超时，未导出轨迹')
        result=rf.result().result
        print('GUN_PREPOSE_PLAN_RESULT',result.error_code.val)
        print('GUN_PREPOSE_LINK6_TARGET_MM',link6.tolist())
        print('GUN_PREPOSE_JOINTS_RAD',goal.tolist())
        if result.error_code.val!=1: raise RuntimeError('充电枪预备位姿规划失败')
        jt=result.planned_trajectory.joint_trajectory
        if not jt.points:raise RuntimeError('MoveIt返回空轨迹')
        target=np.eye(4);target[:3,:3]=Rtcp;target[:3,3]=tcp_pos/1000.
        evidence=self.check_endpoint(jt.joint_names,jt.points[-1].positions,target)
        out.update(planning_ready=True,planning_status='validated',endpoint_validation=evidence)
        self.report=out;atomic_json(REPORT,out)
        print(json.dumps(out,indent=2,ensure_ascii=False),flush=True)
        print('[规划完成] planning_ready=true；仅完成预测终点验收，尚未执行运动。',flush=True)
        payload={'stage':'gun','schema_version':2,'plan_id':f"gun-{time.time_ns()}",
                 'endpoint_validation':evidence,
                 'planned_at':time.time(),'camera_plan_id':out['camera_plan_id'],
                 'camera_request_id':out['camera_request_id'],
                 'pose_source':out['pose_source'],
                 'lateral_compensation':out['lateral_compensation'],
                 'fusion':out['fusion'], 'T_base_from_socket':out['T_base_from_socket'],
                 'cycle_id':out['cycle_id'],'target_stages':out['target_stages'],
                 'angular_compensation_axis':out['angular_compensation_axis'],
                 'pose_selection':out.get('pose_selection'),
                 'selected_distance_m':PREPOSE_DISTANCE_M,'insertion_depth_mm':INSERTION_DEPTH_M*1000,'planning_ready':True,
                 'joint_names':list(jt.joint_names),
                 'points':[{'positions':list(p.positions),'velocities':list(p.velocities),
                            'accelerations':list(p.accelerations),
                            'time_from_start':{'sec':p.time_from_start.sec,
                                               'nanosec':p.time_from_start.nanosec}}
                           for p in jt.points]}
        serialized=json.dumps(payload,indent=2)
        for destination in (GUN_TRAJ_FILE,PENDING_TRAJECTORY):
            temporary=destination.with_name(destination.name+'.'+uuid.uuid4().hex+'.tmp')
            temporary.write_text(serialized)
            temporary.replace(destination)
        msg=DisplayTrajectory(); msg.model_id='nova2_robot'; msg.trajectory_start=result.trajectory_start; msg.trajectory=[result.planned_trajectory]
        self.display_msg=msg
        self.pub.publish(msg)
        self.create_timer(1.0, lambda: (self.pub.publish(self.display_msg), self.publish_planned_tcp()))
        rclpy.spin(self)
        return 0


def main():
    rclpy.init(); n=Planner()
    try: rc=n.plan()
    except Exception as e:
        n.report.update(planning_ready=False,planning_status='failed',error=str(e))
        atomic_json(REPORT,n.report)
        PENDING_TRAJECTORY.unlink(missing_ok=True);GUN_TRAJ_FILE.unlink(missing_ok=True)
        n.get_logger().error(f'只规划失败，未发送运动: {e}');rc=2
    n.destroy_node(); rclpy.shutdown(); return rc
if __name__=='__main__': sys.exit(main())

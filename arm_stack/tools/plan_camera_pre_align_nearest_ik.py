#!/usr/bin/env python3
import json, os, re, sys, time
import uuid
from plan_flow_guard import begin, finish
from stable_pose_selection import collect
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from moveit_msgs.msg import Constraints, JointConstraint, DisplayTrajectory, RobotState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation
from dobot_msgs_v4.srv import GetPose

POSE_JSON=Path('/opt/charging-robot-local/charging_socket_vision/results/v3_diagnostics/latest_pose_readonly.json')
POSE_CACHE=Path('/opt/charging-robot-local/charging_socket_vision/results/realtime_runs/camera_target_pose_cache.json')
HAND_EYE=Path('/opt/charging-robot-local/charging_robot_tools/calibration/hand_eye_calibration_result_ros_24mm.npz')
BASE='base_link'; GROUP='nova2_group'; EE='Link6'; DIST=.140
DISTANCES=(.140,.130)
JOINTS=['joint1','joint2','joint3','joint4','joint5','joint6']
CAM_TRAJ_FILE=Path('/opt/charging-robot-local/charging_socket_vision/results/realtime_runs/pending_camera_trajectory.json')
PENDING_TRAJ_FILE=CAM_TRAJ_FILE.with_name('pending_trajectory.json')
SOCKET_TARGET_FILE=CAM_TRAJ_FILE.with_name('latest_camera_socket_target.json')
REQUEST=CAM_TRAJ_FILE.with_name('latest_camera_plan_request.json')

def rot_dobot(rx,ry,rz):
    rx,ry,rz=np.deg2rad([rx,ry,rz]); cx,sx=np.cos(rx),np.sin(rx); cy,sy=np.cos(ry),np.sin(ry); cz,sz=np.cos(rz),np.sin(rz)
    return np.array([[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx],[sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx],[-sy,cy*sx,cy*cx]])

def joint_constraints(vals):
    c=Constraints()
    for n,v in zip(JOINTS,vals):
        j=JointConstraint(); j.joint_name=n; j.position=float(v); j.tolerance_above=.01; j.tolerance_below=.01; j.weight=1.0; c.joint_constraints.append(j)
    return c

class Planner(Node):
    def __init__(self):
        super().__init__('camera_pre_align_nearest_ik_plan_only')
        self.getpose=self.create_client(GetPose,'/dobot_bringup_ros2/srv/GetPose')
        self.ik=self.create_client(GetPositionIK,'/compute_ik')
        self.validity=self.create_client(GetStateValidity,'/check_state_validity')
        self.planning_ready=False
        self.move=ActionClient(self,MoveGroup,'/move_action')
        self.current_joints=None
        self.joint_sub=self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos=QoSProfile(depth=1); qos.reliability=ReliabilityPolicy.RELIABLE; qos.durability=DurabilityPolicy.TRANSIENT_LOCAL
        self.display_pub=self.create_publisher(DisplayTrajectory,'/display_planned_path',qos)
    def joint_cb(self, msg):
        vals=dict(zip(msg.name, msg.position))
        if all(n in vals for n in JOINTS):
            self.current_joints=np.array([vals[n] for n in JOINTS], dtype=float)

    def current_pose(self):
        if not self.getpose.wait_for_service(8): raise RuntimeError('GetPose unavailable')
        f=self.getpose.call_async(GetPose.Request(user=0,tool=0)); rclpy.spin_until_future_complete(self,f,timeout_sec=8); res=f.result()
        if res is None or res.res!=0: raise RuntimeError(f'GetPose failed: {res}')
        vals=[float(x) for x in re.findall(r'[-+]?\d+(?:\.\d+)?',res.robot_return)]
        if len(vals)<6: raise RuntimeError(res.robot_return)
        return np.asarray(vals[:3])/1000., np.asarray(vals[3:6]), np.array([float(x) for x in vals[:6]])
    def target_pose(self):
        _,_,robot=self.current_pose()
        robot_mm = robot.copy()
        robot[:3] = robot[:3] / 1000.0
        with np.load(HAND_EYE) as d:
            Tgc=np.eye(4); Tgc[:3,:3]=np.asarray(d['R_cam2gripper'],float); Tgc[:3,3]=np.asarray(d['t_cam2gripper'],float).reshape(3)
        if np.linalg.norm(Tgc[:3,3])>2:Tgc[:3,3]/=1000.
        def read_icp():
            d=json.loads(POSE_JSON.read_text())
            if d.get('pose_valid') is not True or d.get('pose_fresh') is not True or d.get('quality_level')!='full_geometry':
                raise ValueError('invalid_visual:'+str(d.get('failure_reason',d.get('status'))))
            return {'stamp':d.get('measurement_time'),'matrix':d.get('T_camera_from_socket_raw'),'payload':d}
        chosen, selection=collect(read_icp,'camera',CAM_TRAJ_FILE.with_name('latest_camera_pose_selection.json'))
        vision=chosen['payload']
        _,_,robot_after=self.current_pose()
        if np.linalg.norm(robot_after[:3]-robot_mm[:3])>0.5 or np.degrees(Rotation.from_matrix(rot_dobot(*robot_mm[3:6]).T@rot_dobot(*robot_after[3:6])).magnitude())>0.5:
            raise RuntimeError('稳定取值期间机械臂移动，请保持静止后重新cam-plan')
        sampled_at=time.time()
        measurement=vision.get('measurement_time')
        if not vision.get('pose_valid') or not vision.get('pose_fresh') or vision.get('quality_level')!='full_geometry':
            raise RuntimeError(f"ICP not valid complete geometry: {vision.get('status')}")
        if measurement is None or not np.isfinite(float(measurement)) or not 0<=time.time()-float(selection['latest_measurement_time'])<=1.0:
            raise RuntimeError('ICP measurement expired or missing; need a fresh frame within 1 second')
        matrix=np.asarray(vision.get('T_camera_from_socket_raw'),float)
        if matrix.shape!=(4,4) or not np.isfinite(matrix).all() or not np.allclose(matrix[3],[0,0,0,1]):
            raise RuntimeError('ICP transform malformed')
        if not np.allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),atol=1e-3) or abs(np.linalg.det(matrix[:3,:3])-1)>1e-3:
            raise RuntimeError('ICP rotation malformed')
        self.planning_ready=vision.get('planning_ready') is True
        if not self.planning_ready:
            self.get_logger().warn('ICP planning_ready=false: plan will be saved with execution blocked; no control cache export')
        # Freeze geometry once for the whole distance search.
        cache=dict(vision); cache['pose_selection']=selection; cache['_cache_source']='cam-plan'; cache['_cache_source_file']=str(POSE_JSON)
        cache['_measurement_sampled_at']=sampled_at
        import time as _time; cache['_cached_at_unix']=_time.time()
        Tcs=np.asarray(vision['T_camera_from_socket_raw'],float)
        Tbg=np.eye(4); Tbg[:3,:3]=rot_dobot(*robot[3:6]); Tbg[:3,3]=robot[:3]
        Tbs=Tbg@Tgc@Tcs; self.frozen_Tbs=Tbs.copy(); self.frozen_Tgc=Tgc.copy(); Rs=Tbs[:3,:3]; o=Tbs[:3,3]; cam=o+Rs[:,2]*DIST
        chosen['T_base_from_socket']=Tbs.tolist()
        chosen['robot_pose_tool0_mm_deg']=robot_mm.tolist()
        from stable_pose_selection import atomic_json
        atomic_json(CAM_TRAJ_FILE.with_name('latest_camera_pose_selection.json'), dict(status='accepted', stage='camera', selected_at=time.time(), statistics=selection, selected=chosen, source='icp_cam_plan_base_target'))
        Rc=np.column_stack((Rs[:,0],-Rs[:,1],-Rs[:,2])); Tbc=np.eye(4); Tbc[:3,:3]=Rc; Tbc[:3,3]=cam
        Tgoal=Tbc@np.linalg.inv(Tgc)
        # Retain selected camera prepose as diagnostic metadata.
        cache['T_base_from_camera_prepose']=Tbc.tolist()
        cache['camera_prepose_origin_base_m']=Tbc[:3,3].tolist()
        cache['camera_prepose_plus_z_base']=Tbc[:3,2].tolist()
        cache['camera_prepose_distance_m']=DIST
        self.pose_cache=cache
        p=PoseStamped(); p.header.frame_id=BASE; p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,Tgoal[:3,3]); q=Rotation.from_matrix(Tgoal[:3,:3]).as_quat(); p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=map(float,q)
        self.get_logger().info(f'ICP fitness={vision.get("fitness")} rmse={vision.get("rmse_m")} camera_target={cam.tolist()}')
        self.get_logger().info(f'Link6 target={Tgoal[:3,3].tolist()} rpy={Rotation.from_matrix(Tgoal[:3,:3]).as_euler("xyz",degrees=True).tolist()}')
        return p, robot[:6]
    def check_state(self, joints):
        if not self.validity.wait_for_service(timeout_sec=3): raise RuntimeError('check_state_validity unavailable')
        req=GetStateValidity.Request(); req.group_name=GROUP
        req.robot_state.joint_state.name=JOINTS
        req.robot_state.joint_state.position=list(map(float,joints))
        f=self.validity.call_async(req); rclpy.spin_until_future_complete(self,f,timeout_sec=4)
        res=f.result()
        if res is None: raise RuntimeError('state validity no response')
        contacts=[f'{c.contact_body_1}/{c.contact_body_2}' for c in res.contacts]
        return res.valid, contacts

    def solve_ik(self, pose, seed):
        # Query the active MoveIt model rather than assuming +/-360 degree limits.
        from rcl_interfaces.srv import GetParameters
        import xml.etree.ElementTree as ET
        cli=self.create_client(GetParameters,'/move_group/get_parameters')
        if not cli.wait_for_service(timeout_sec=3): raise RuntimeError('MoveIt model parameters unavailable')
        request=GetParameters.Request(); request.names=['robot_description']
        f=cli.call_async(request); rclpy.spin_until_future_complete(self,f,timeout_sec=4)
        response=f.result()
        if response is None or not response.values: raise RuntimeError('No active robot_description')
        root=ET.fromstring(response.values[0].string_value)
        bounds=[]
        for name in JOINTS:
            joint=root.find("joint[@name='"+name+"']")
            if joint is None: raise RuntimeError('Missing joint in model: '+name)
            if joint.get('type')=='continuous': bounds.append((-np.inf,np.inf))
            else:
                limit=joint.find('limit')
                bounds.append((float(limit.get('lower')),float(limit.get('upper'))))
        if not self.ik.wait_for_service(timeout_sec=4): raise RuntimeError('compute_ik unavailable')
        guesses=[seed.copy()]
        # Nearby elbow/wrist seeds; the requested Cartesian pose stays identical.
        for index in (2,1,3,4,5):
            for sign in (-1,1):
                guess=seed.copy(); guess[index]+=sign*np.deg2rad(25)
                guess[index]=np.clip(guess[index],*bounds[index]); guesses.append(guess)
        for sign in (-1,1):
            guess=seed.copy(); guess[1]+=sign*.35; guess[2]-=sign*.35
            for i in range(6): guess[i]=np.clip(guess[i],*bounds[i])
            guesses.append(guess)
        rng=np.random.default_rng(42)
        for _ in range(50):
            guess=seed+rng.uniform(-1.2,1.2,6)
            for i in range(6): guess[i]=np.clip(guess[i],*bounds[i])
            guesses.append(guess)
        candidates=[]; rejected_contacts={}; deadline=time.monotonic()+15
        for attempt,guess in enumerate(guesses):
            if time.monotonic()>deadline: break
            req=GetPositionIK.Request(); ir=req.ik_request
            ir.group_name=GROUP; ir.ik_link_name=EE; ir.pose_stamped=pose
            ir.robot_state.joint_state.name=JOINTS
            ir.robot_state.joint_state.position=list(map(float,guess))
            ir.timeout.nanosec=250000000; ir.avoid_collisions=False
            f=self.ik.call_async(req); rclpy.spin_until_future_complete(self,f,timeout_sec=1)
            res=f.result()
            if res is None or res.error_code.val!=1: continue
            vals=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
            sol=np.array([vals[name] for name in JOINTS],float)
            if not np.isfinite(sol).all(): continue
            # Enumerate equivalent angles inside the active URDF limits.
            for i,(low,high) in enumerate(bounds):
                center=int(round((seed[i]-sol[i])/(2*np.pi)))
                equivalents=[sol[i]+2*np.pi*k for k in range(center-2,center+3)
                             if low-1e-8<=sol[i]+2*np.pi*k<=high+1e-8]
                if not equivalents: break
                sol[i]=min(equivalents,key=lambda value:abs(value-seed[i]))
            else:
                delta=np.abs(sol-seed)
                if np.any(delta>np.deg2rad(90)): continue
                if any(np.max(np.abs(sol-old[1]))<1e-3 for old in candidates): continue
                valid,contacts=self.check_state(sol)
                if not valid:
                    for contact in contacts: rejected_contacts[contact]=rejected_contacts.get(contact,0)+1
                    continue
                cost=float(np.linalg.norm(delta)+.5*np.max(delta))
                candidates.append((cost,sol.copy()))
                self.get_logger().info(f'Near IK attempt={attempt} actual_delta_deg={np.rad2deg(sol-seed).tolist()}')
                # A small local solution is sufficient; avoid unnecessary service load.
                if np.max(delta)<np.deg2rad(50): break
        if not candidates:
            raise RuntimeError(f'No collision-free IK within 90deg found; nearby collision counts={rejected_contacts}; endpoint unchanged')
        sol=min(candidates,key=lambda item:item[0])[1]
        self.get_logger().info(f'Selected near IK={sol.tolist()} actual_delta_deg={np.rad2deg(sol-seed).tolist()}')
        return sol
    def plan_candidate(self,pose,seed):
        goalj=self.solve_ik(pose,seed)
        valid,contacts=self.check_state(goalj)
        if not valid: raise RuntimeError(f'IK goal state invalid: {contacts}')
        if not self.move.wait_for_server(10): raise RuntimeError('MoveGroup unavailable')
        goal=MoveGroup.Goal(); goal.request.group_name=GROUP; goal.request.start_state.joint_state.name=JOINTS; goal.request.start_state.joint_state.position=list(map(float,seed)); goal.request.goal_constraints=[joint_constraints(goalj)]; goal.request.allowed_planning_time=10.; goal.request.num_planning_attempts=5; goal.request.max_velocity_scaling_factor=.15; goal.request.max_acceleration_scaling_factor=.10; goal.request.pipeline_id='ompl'; goal.request.planner_id='RRTConnect'; goal.planning_options.plan_only=True; goal.planning_options.replan=False
        f=self.move.send_goal_async(goal); rclpy.spin_until_future_complete(self,f,timeout_sec=15); h=f.result()
        if h is None or not h.accepted: raise RuntimeError('planning goal rejected')
        rf=h.get_result_async(); rclpy.spin_until_future_complete(self,rf,timeout_sec=30); result=rf.result().result
        self.get_logger().info(f'PLAN_RESULT code={result.error_code.val} time={result.planning_time:.3f}s')
        jt=result.planned_trajectory.joint_trajectory; self.get_logger().info(f'points={len(jt.points)}')
        if jt.points:
            self.get_logger().info(f'first={list(jt.points[0].positions)}'); self.get_logger().info(f'last={list(jt.points[-1].positions)}')
        if result.error_code.val!=1: raise RuntimeError(f'Planning failed code={result.error_code.val}')
        jt=result.planned_trajectory.joint_trajectory
        names=list(jt.joint_names)
        if set(names)!=set(JOINTS) or not jt.points:
            raise RuntimeError('Invalid planned trajectory joints/points')
        origin=np.array([seed[JOINTS.index(name)] for name in names])
        positions=np.array([point.positions for point in jt.points],float)
        if not np.isfinite(positions).all(): raise RuntimeError('Non-finite planned trajectory')
        excursion=np.rad2deg(np.max(np.abs(positions-origin),axis=0))
        travel=np.rad2deg(np.sum(np.abs(np.diff(positions,axis=0)),axis=0))
        self.get_logger().info(f'Trajectory excursion_deg={excursion.tolist()} travel_deg={travel.tolist()}')
        if np.any(excursion>90.) or np.any(travel>120.):
            raise RuntimeError('Reject winding trajectory: joint excursion >90deg or total travel >120deg')
        return result

    def run(self):
        self.request_id=os.environ.get('CHARGE_PLAN_REQUEST') or begin()
        if REQUEST.exists():
            request_state=json.loads(REQUEST.read_text())
            if request_state.get('request_id')==self.request_id and request_state.get('status')=='planning':
                if PENDING_TRAJ_FILE.exists():
                    try:
                        old_pending=json.loads(PENDING_TRAJ_FILE.read_text())
                    except (OSError, json.JSONDecodeError):
                        old_pending={}
                    if old_pending.get('stage')=='camera':
                        PENDING_TRAJ_FILE.unlink(missing_ok=True)
                if CAM_TRAJ_FILE.exists():
                    CAM_TRAJ_FILE.unlink(missing_ok=True)
        pose,_=self.target_pose()
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_joints is not None:
                break
        if self.current_joints is None:
            raise RuntimeError('No valid /joint_states received')
        seed=self.current_joints.copy()
        self.get_logger().info(f'Planning start joints from /joint_states={seed.tolist()}')
        valid,contacts=self.check_state(seed)
        if not valid: raise RuntimeError(f'Current start state invalid: {contacts}')
        failures=[]; result=None
        for distance in DISTANCES:
            # Same frozen socket, hand-eye and joint start for every candidate.
            S=self.frozen_Tbs; G=self.frozen_Tgc
            C=np.eye(4); C[:3,:3]=np.column_stack((S[:3,0],-S[:3,1],-S[:3,2]))
            C[:3,3]=S[:3,3]+S[:3,2]*distance
            target=C@np.linalg.inv(G)
            pose=PoseStamped(); pose.header.frame_id=BASE
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,target[:3,3])
            q=Rotation.from_matrix(target[:3,:3]).as_quat()
            pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=map(float,q)
            self.get_logger().info(f'Trying coaxial camera distance={distance*1000:.0f}mm')
            try:
                result=self.plan_candidate(pose,seed)
            except RuntimeError as error:
                failures.append({'distance_m':distance,'reason':str(error)})
                self.get_logger().warn(f'Rejected {distance*1000:.0f}mm: {error}')
                continue
            self.pose_cache.update(T_base_from_camera_prepose=C.tolist(),
                camera_prepose_origin_base_m=C[:3,3].tolist(),
                camera_prepose_plus_z_base=C[:3,2].tolist(),
                camera_prepose_distance_m=distance,distance_search_rejections=failures)
            self.get_logger().info(f'SELECTED_CAMERA_DISTANCE_MM={distance*1000:.0f}')
            break
        if result is None:
            raise RuntimeError(f'No acceptable coaxial plan at 140/130mm: {failures}')
        jt=result.planned_trajectory.joint_trajectory
        payload={'stage':'camera','joint_names':list(jt.joint_names),'points':[{'positions':list(p.positions),'velocities':list(p.velocities),'accelerations':list(p.accelerations),'time_from_start':{'sec':p.time_from_start.sec,'nanosec':p.time_from_start.nanosec}} for p in jt.points]}
        payload.update(schema_version=2,plan_id=f"camera-{time.time_ns()}",planned_at=time.time(),request_id=self.request_id,
            planning_ready=self.planning_ready,
            planning_block_reason=self.pose_cache.get('planning_block_reason'),
            selected_distance_m=self.pose_cache['camera_prepose_distance_m'],
            distance_search_rejections=self.pose_cache.get('distance_search_rejections',[]),
            measurement_time=self.pose_cache.get('measurement_time'),
            pose_valid=self.pose_cache.get('pose_valid'),
            pose_fresh=self.pose_cache.get('pose_fresh'),
            quality_level=self.pose_cache.get('quality_level'),
            measurement_sampled_at=self.pose_cache.get('_measurement_sampled_at'),
            pose_selection=self.pose_cache.get('pose_selection'),
            T_base_from_socket=self.frozen_Tbs.tolist(),
            camera_plan_robot_pose_tool0=self.pose_cache.get('robot_pose_tool0_mm_deg'))
        cycle=json.loads((CAM_TRAJ_FILE.parent/'workflow_cycle.json').read_text())
        selection_path=CAM_TRAJ_FILE.parent/'latest_camera_pose_selection.json'
        selection_record=json.loads(selection_path.read_text())
        from stable_pose_selection import atomic_json
        selection_record.update(cycle_id=cycle['cycle_id'],camera_plan_id=payload['plan_id'],
                                camera_request_id=self.request_id,planning_ready=self.planning_ready)
        atomic_json(selection_path,selection_record)
        # Persist exactly the selected, checked trajectory even for preview plans.
        serialized=json.dumps(payload,indent=2)
        for destination in (CAM_TRAJ_FILE, PENDING_TRAJ_FILE):
            temporary=destination.with_name(destination.name+'.'+uuid.uuid4().hex+'.tmp')
            temporary.write_text(serialized)
            os.replace(temporary,destination)
        socket_target={
            'schema_version':1,
            'stage':'camera_socket_target',
            'request_id':self.request_id,
            'plan_id':payload['plan_id'],
            'camera_plan_at':payload['planned_at'],
            'measurement_time':payload.get('measurement_time'),
            'T_base_from_socket':self.frozen_Tbs.tolist(),
        }
        target_tmp=SOCKET_TARGET_FILE.with_name(SOCKET_TARGET_FILE.name+'.'+uuid.uuid4().hex+'.tmp')
        target_tmp.write_text(json.dumps(socket_target,indent=2))
        os.replace(target_tmp,SOCKET_TARGET_FILE)
        if self.planning_ready:
            POSE_CACHE.write_text(json.dumps(self.pose_cache,indent=2))
        finish(self.request_id,'complete',plan_id=payload['plan_id'],planning_ready=self.planning_ready,
               planning_block_reason=payload['planning_block_reason'])
        self.get_logger().info(f"PLAN_STATUS: planning=SUCCESS execution={'ELIGIBLE' if self.planning_ready else 'BLOCKED'} request_id={self.request_id}")
        self.get_logger().info(f"SAVED_PLAN id={payload['plan_id']} distance_mm={payload['selected_distance_m']*1000:.0f} planning_ready={self.planning_ready}")
        display=DisplayTrajectory(); display.model_id='nova2_robot'; display.trajectory_start=result.trajectory_start; display.trajectory=[result.planned_trajectory]
        self.display_msg=display
        self.display_pub.publish(display)
        self.create_timer(1.0, lambda: self.display_pub.publish(self.display_msg))
        return True

def main():
    rclpy.init(); n=Planner()
    try:
        ok=n.run(); rc=0 if ok else 2
        # Keep publishing the current plan until the next plan command stops us.
        if ok: rclpy.spin(n)
    except KeyboardInterrupt: rc=0
    except Exception as e:
        n.get_logger().error(str(e)); rc=3
        if hasattr(n,'request_id'):
            try: finish(n.request_id,'failed',error=str(e))
            except Exception as status_error: n.get_logger().error(str(status_error))
    n.destroy_node(); rclpy.shutdown(); return rc
if __name__=='__main__': sys.exit(main())

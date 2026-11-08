"""ROS I/O. Constructed only in online mode; no nodes are launched here."""
import math
import queue
import threading
import time
import uuid
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32
from power_status import PowerStatus, Power48Status
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from ychd_robot_keyboard.ychd_keyboard import moveBindings as YCHD_BINDINGS
from teleop import velocity, SPEEDS_M_S
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener, TransformException
from model import yaw_of
from docking_state import StopObservation, readiness
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


from arm_tools_bridge import ArmToolsBridge
from return_bridge import ReturnBridge
from nav2_pause_bridge import Nav2PauseBridge
from runtime_provenance import panel_identity
from return_map import grid_digest
from return_runtime import read_alignment_tolerance,alignment_timing


from stop_recovery import StopRecoveryBridge

class Bridge(StopRecoveryBridge, ArmToolsBridge, Nav2PauseBridge, ReturnBridge, Node):
    def __init__(self,docking_hold=False, *, recovery_required=False, ros_args=None):
        rclpy.init(args=ros_args)
        super().__init__('charging_navigation_panel')
        self.lock=threading.Lock(); self.commands=queue.Queue(); self.events=queue.Queue()
        self.init_nav2_pause()
        self.grid=None; self.grid_seq=0; self.scan=None; self.path=[]
        self.scan_times=deque(maxlen=40); self.odom_time=0.; self.velocity=(0.,0.)
        self.pose=None; self.pose_time=0.; self.scan_world=[]; self.nodes=[]; self.graph_ready=False
        self.initial_ready=False;self.nav_ready=False
        self.localization_epoch=uuid.uuid4().hex;self.navigation_id=None
        self.pose_stamp=None;self.pose_received_at=0.
        self.power=PowerStatus();self.power48=Power48Status()
        self.status='未发送目标'; self.distance=None; self.busy=False
        self.goal=None; self.cancel_requested=False
        self.goal_status=None;self.goal_target=None;self.goal_finished_at=None
        self.stop_observation=StopObservation();self.docking_hold=docking_hold
        self.manual=False; self.manual_target=(0.,0.); self.manual_stamp=0.
        self.manual_stop_pending=False; self.server_active=False;self.software_stop=False;self.motion_recovery_required=False
        self.init_return()
        self.alignment_tolerance=read_alignment_tolerance(Path(__file__).parent/'config/nav2_ackermann.yaml')
        # Legacy zero requests are routed to a disconnected input; the arbiter owns chassis output.
        self.manual_pub=self.create_publisher(Twist,'/charging_control/legacy_zero',1)
        self.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',self.nav_status,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.05,self.manual_tick)
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        mapq=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,mapq)
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        from std_msgs.msg import String
        self.create_subscription(String,'/charging_control/raw_odometry',self.on_raw_odom,10)
        self.create_subscription(Float32,'/PowerVoltage',self.on_power,qos_profile_sensor_data)
        self.create_subscription(Float32,'/PowerVoltage48V',self.on_power48,qos_profile_sensor_data)
        self.create_subscription(NavPath,'/plan',self.on_path,10)
        self.initial=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.action=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.cancel_all_client=self.create_client(CancelGoal,'/navigate_to_pose/_action/cancel_goal')
        self.create_timer(.1,self.tick); self.create_timer(1.,self.discover)
        self.ros_executor=SingleThreadedExecutor(); self.ros_executor.add_node(self)
        self.thread=threading.Thread(target=self.ros_executor.spin,daemon=True);self.thread.start()

    def nav_status(self,msg):
        with self.lock:self.server_active=any(g.status in (1,2,3) for g in msg.status_list)

    def emergency_stop(self,reason='operator_stop',detail='用户请求底盘停车'):
        with self.lock:
            self.latch_software_stop(reason,detail);self.stop_reset_pending=None;self.reset_arbiter=False;self.manual=False;self.manual_target=(0.,0.)
            self.manual_stop_pending=False;self.cancel_requested=True
            self.manual_pub.publish(Twist())
        self.control_tick()
        self.commands.put(('cancel',()))
        self.event('software_stop_requested',zero_velocity_requested=True,requested_source=reason,requested_detail=detail,first_cause=self.stop_cause)

    def enable_manual(self):
        with self.lock:
            self.return_guard()
            if getattr(self,'docking_hold',False):raise ValueError('对接锁定中，请先现场确认机械臂已收拢并解除对接锁')
            if self.software_stop or self.motion_recovery_required:raise ValueError('底盘运动未放行，请先确认底盘就绪')
            if self.busy or self.server_active or self.manual_stop_pending:
                raise ValueError('导航任务或停车请求尚未结束')
            if time.monotonic()-self.odom_time>1.:
                raise ValueError('尚无新鲜底盘里程计')
            self.cancel_requested=False;self.stop_cause=None
            self.manual=True;self.manual_target=(0.,0.);self.manual_stamp=time.monotonic()
            self.goal_status=None;self.goal_target=None;self.goal_finished_at=None
        self.event('manual_enabled',source='ychd_robot_keyboard.moveBindings -> cmd_vel -> ychd_robot_node')

    def update_manual(self,keys,speed):
        if speed not in SPEEDS_M_S:raise ValueError('无效的手动速度档位')
        with self.lock:
            if not self.manual:return False
            target=velocity(keys,speed,YCHD_BINDINGS)
            changed=target!=self.manual_target
            self.manual_target=target
            self.manual_stamp=time.monotonic()
            observed=self.velocity;observed_age=self.manual_stamp-self.odom_time
        if changed:
            self.event('manual_command',keys=sorted(keys),target_v_m_s=target[0],target_w_rad_s=target[1],
                       odom_v_m_s=observed[0],odom_w_rad_s=observed[1],odom_age_s=observed_age)
        return True

    def disable_manual(self,reason='用户退出'):
        with self.lock:
            if not self.manual:return
            self.manual=False;self.manual_target=(0.,0.);self.manual_stop_pending=True
        self.event('manual_disabled',reason=reason)

    def manual_tick(self):
        self.control_tick()

    def reset_view(self):
        with self.lock:
            self.grid=None;self.grid_seq+=1;self.pose=None;self.pose_time=0.;self.path=[]
            self.goal_status=None;self.goal_target=None;self.goal_finished_at=None
        self.commands.put(('reset',()))

    def on_map(self,msg):
        with self.lock:
            self.grid=msg;self.grid_seq+=1
            try:self.map_grid_sha=grid_digest(msg)
            except (ValueError,TypeError):self.map_grid_sha=None
    def on_scan(self,msg):
        with self.lock: self.scan=msg;self.scan_times.append(time.monotonic())
    def on_odom(self,msg):
        with self.lock:
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            self.odom_frame_valid=(msg.header.frame_id=='odom_combined' and msg.child_frame_id=='base_footprint' and
                (self.odom_source_stamp is None or stamp>self.odom_source_stamp))
            self.odom_frame=msg.header.frame_id;self.odom_source_stamp=stamp
            self.odom_time=time.monotonic()
            self.velocity=(msg.twist.twist.linear.x,msg.twist.twist.angular.z)
            self.stop_observation.update(*self.velocity,self.odom_time)
    def on_power(self,msg):
        with self.lock:self.power.update(msg.data,time.monotonic())

    def on_power48(self,msg):
        with self.lock:self.power48.update(msg.data,time.monotonic())

    def on_path(self,msg):
        if msg.header.frame_id=='map':
            with self.lock:self.path=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
    def discover(self):
        nodes=self.get_node_names();initial_ready=self.count_subscribers('/initialpose')>0 and 'amcl' in nodes
        nav_ready=self.action.server_is_ready()
        with self.lock:self.nodes=nodes;self.graph_ready=True;self.initial_ready=initial_ready;self.nav_ready=nav_ready
    def event(self,kind,**data):self.events.put({'event':kind,**data})

    def tick(self):
        self.tick_nav2_pause()
        try:
            t=self.tf.lookup_transform('map','base_footprint',rclpy.time.Time())
            age=(self.get_clock().now().nanoseconds-(t.header.stamp.sec*10**9+t.header.stamp.nanosec))/1e9
            if 0 <= age <= 1.:
                raw_alignment=self.tf.lookup_transform('map','odom_combined',rclpy.time.Time())
                alignment_stamp=raw_alignment.header.stamp.sec+raw_alignment.header.stamp.nanosec/1e9
                alignment_age=self.get_clock().now().nanoseconds/1e9-alignment_stamp
                paired_time=rclpy.time.Time.from_msg(t.header.stamp)
                alignment=self.tf.lookup_transform('map','odom_combined',paired_time)
                odom_tf=self.tf.lookup_transform('odom_combined','base_footprint',paired_time)
                def planar(tf):
                    return (tf.transform.translation.x,tf.transform.translation.y,yaw_of(tf.transform.rotation))
                with self.lock:
                    now=time.monotonic()
                    if getattr(self,'alignment_stamp',None)!=alignment_stamp:
                        self.alignment_stamp=alignment_stamp;self.alignment_received_at=now
                    adjusted_age,clock_valid,source_age=alignment_timing(
                        self.get_clock().now().nanoseconds/1e9,alignment_stamp,now-self.alignment_received_at,self.alignment_tolerance)
                    self.paired_pose=dict(odom_pose=planar(odom_tf),map_to_odom=planar(alignment),
                        latest_map_to_odom=planar(raw_alignment),
                        odom_tf_age=age,alignment_age=adjusted_age,alignment_source_age=source_age,
                        paired_at=now,alignment_future_valid=clock_valid)

                    self.pose=(t.transform.translation.x,t.transform.translation.y,yaw_of(t.transform.rotation))
                    stamp=t.header.stamp.sec+t.header.stamp.nanosec/1e9
                    if stamp!=self.pose_stamp:self.pose_stamp=stamp;self.pose_received_at=time.monotonic()
                    self.pose_time=time.monotonic()-age
        except TransformException:pass
        with self.lock:scan=self.scan
        points=[]
        if scan:
            try:
                tf=self.tf.lookup_transform('map',scan.header.frame_id,rclpy.time.Time.from_msg(scan.header.stamp))
                yaw=yaw_of(tf.transform.rotation);c,s=math.cos(yaw),math.sin(yaw)
                for i in range(0,len(scan.ranges),3):
                    r=scan.ranges[i]
                    if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                        a=scan.angle_min+i*scan.angle_increment;x,y=r*math.cos(a),r*math.sin(a)
                        points.append((tf.transform.translation.x+c*x-s*y,tf.transform.translation.y+s*x+c*y))
            except TransformException:pass
        with self.lock:self.scan_world=points
        while not self.commands.empty():
            kind,args=self.commands.get_nowait()
            if kind=='reset':
                self.tf.clear()
                with self.lock:
                    self.grid=None;self.grid_seq+=1;self.pose=None;self.pose_time=0.;self.path=[]
                    self.localization_epoch=uuid.uuid4().hex;self.pose_stamp=None;self.pose_received_at=0.
                    self.goal_status=None;self.goal_target=None;self.goal_finished_at=None;self.status='未发送目标'
            elif kind=='initial':self.set_initial(*args)
            elif kind=='goal':self.send_goal(*args)
            elif kind=='cancel':self.cancel()

    def set_initial(self,x,y,yaw):
        with self.lock:
            if self.docking_hold:return
            if self.return_active():
                self.event('initial_pose_blocked_during_return');return
            self.localization_epoch=uuid.uuid4().hex
            self.goal_status=None;self.goal_target=None;self.goal_finished_at=None
        msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.pose.position.x=x;msg.pose.pose.position.y=y
        msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
        msg.pose.covariance[0]=msg.pose.covariance[7]=.25;msg.pose.covariance[35]=.0685
        self.initial.publish(msg);self.event('initial_pose',x_m=x,y_m=y,yaw_rad=yaw)

    def request_goal(self,pose,request_id=None):
        with self.lock:
            self.return_guard()
            if self.nav2_pause_gate.phase!='idle':raise ValueError('Nav2导航已暂停或暂停未完成；不会自动恢复，请结束本轮后关闭并重开面板及定位导航')
            if getattr(self,'docking_hold',False):raise ValueError('对接锁定中，请先现场确认机械臂已收拢并解除对接锁')
            if self.software_stop or self.motion_recovery_required:raise ValueError('底盘运动未放行，请先确认底盘就绪')
            if self.manual or self.manual_stop_pending:raise ValueError('请先退出键盘控制并等待停车命令发出')
            if self.busy or self.server_active:raise ValueError('已有导航请求正在处理，请先取消或等待完成')
            self.stop_cause=None
            self.navigation_id=request_id or uuid.uuid4().hex
            self.busy=True;self.status='等待目标接受';self.cancel_requested=False;self.path=[]
            self.goal_status=None;self.goal_target=tuple(pose);self.goal_finished_at=None
        self.commands.put(('goal',(*pose,self.navigation_id)))

    def send_goal(self,x,y,yaw,request_id):
        with self.lock:
            if request_id!=self.navigation_id:return
            if self.cancel_requested or self.software_stop or self.motion_recovery_required or getattr(self,'docking_hold',False):
                self.busy=False;self.event('queued_goal_blocked_by_stop');return
        if not self.action.server_is_ready():
            with self.lock:self.busy=False;self.status='导航 Action 尚未就绪'
            self.event('goal_unavailable');return
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=self.get_clock().now().to_msg()
        goal.behavior_tree=str(Path(get_package_share_directory('nav2_bt_navigator'))/'behavior_trees'/'navigate_w_replanning_time.xml')
        goal.pose.pose.position.x=x;goal.pose.pose.position.y=y
        goal.pose.pose.orientation.z=math.sin(yaw/2);goal.pose.pose.orientation.w=math.cos(yaw/2)
        self.event('goal_requested',x_m=x,y_m=y,yaw_rad=yaw)
        try:
            with self.lock:
                if self.cancel_requested or self.software_stop or self.motion_recovery_required or getattr(self,'docking_hold',False):
                    self.busy=False;self.event('queued_goal_blocked_by_stop');return
                future=self.action.send_goal_async(goal,feedback_callback=self.feedback)
            future.add_done_callback(lambda result:self.accepted(result,request_id))
        except Exception as error:
            with self.lock:self.status='目标提交状态不确定'
            self.event('goal_error',error=str(error))

    def accepted(self,future,request_id):
        if request_id!=self.navigation_id:return
        try:
            self.goal=future.result()
            if not self.goal.accepted:
                with self.lock:self.busy=False;self.status='目标被拒绝';self.goal_status=None;self.goal_finished_at=None
                self.event('goal_rejected');return
            with self.lock:self.status='导航中'
            self.event('goal_accepted')
            self.goal.get_result_async().add_done_callback(lambda result:self.finished(result,request_id))
            if self.cancel_requested or self.software_stop:self.cancel()
        except Exception as error:
            with self.lock:self.status='目标响应异常，状态未确认'
            self.event('goal_error',error=str(error))

    def feedback(self,msg):
        with self.lock:self.distance=msg.feedback.distance_remaining
    def finished(self,future,request_id):
        if request_id!=self.navigation_id:return
        try:
            result=future.result()
            status={4:'导航成功（Nav2容差，待停稳与现场确认）',5:'导航已取消',6:'导航失败'}.get(result.status,'导航结束状态 '+str(result.status))
            with self.lock:
                self.busy=False;self.status=status;self.distance=None
                self.goal_status=result.status;self.goal_finished_at=time.monotonic()
            self.goal=None;self.event('goal_result',status=result.status,navigation_id=request_id)
        except Exception as error:
            with self.lock:self.status='执行结果异常，状态未确认';self.goal_status=None;self.goal_finished_at=None
            self.event('result_error',error=str(error))
    def cancel(self):
        self.cancel_requested=True
        if not self.goal or not self.goal.accepted or not self.busy:
            if self.server_active:
                if not self.cancel_all_client.service_is_ready():self.event('cancel_error',error='导航取消服务不可用');return
                self.cancel_all_client.call_async(CancelGoal.Request()).add_done_callback(self.cancel_response)
                self.event('cancel_requested',scope='existing_nav2_goals')
            return
        with self.lock:self.status='已请求取消，等待结果'
        self.event('cancel_requested')
        try:self.goal.cancel_goal_async().add_done_callback(self.cancel_response)
        except Exception as error:self.event('cancel_error',error=str(error))
    def cancel_response(self,future):
        try:self.event('cancel_response',return_code=future.result().return_code)
        except Exception as error:self.event('cancel_error',error=str(error))

    def begin_docking(self):
        # Recheck ROS-owned state under the lock; UI confirmation alone is not a gate.
        with self.lock:
            self.return_guard()
            ok,reason=readiness(self._snapshot(time.monotonic()),True)
            if not ok:raise ValueError(reason)
            self.docking_hold=True
            self.manual_pub.publish(Twist())
        self.event('docking_hold_enabled',scope='panel_chassis_only')

    def release_docking(self):
        return ArmToolsBridge.release_docking(self)

    def _snapshot(self,now):
        times=list(self.scan_times)
        paired=dict(self.paired_pose or {})
        if paired:
            elapsed=now-paired.pop('paired_at')
            paired['odom_tf_age']+=elapsed;paired['alignment_age']+=elapsed
            if not paired.pop('alignment_future_valid'):paired['alignment_age']=math.inf
        return dict(nav2_pause=self.nav2_pause_gate.snapshot(),**paired,odom_frame=self.odom_frame,base_frame='base_footprint',
            odom_frame_valid=self.odom_frame_valid,odom_source_stamp=self.odom_source_stamp,map_sha256=self.gui_map_sha,map_grid_sha=getattr(self,'map_grid_sha',None),
            return_state=dict(self.return_state),return_active=self.return_active(),
            **self.raw_snapshot(now),
            runtime_identity=panel_identity(self.localization_epoch,self.arbiter_state),
            arbiter=dict(self.arbiter_state),arbiter_age=now-self.arbiter_at if self.arbiter_at else math.inf,
            navigation_id=self.navigation_id,localization_epoch=self.localization_epoch,
            pose_stamp=self.pose_stamp,pose_received_at=self.pose_received_at,
            stationary_since=self.stop_observation.first,goal_finished_at=self.goal_finished_at,
            grid=self.grid,grid_seq=self.grid_seq,pose=self.pose,pose_age=now-self.pose_time,
            scan_age=now-times[-1] if times else math.inf,
            scan_hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0.,
            odom_age=max(now-self.odom_time,self.get_clock().now().nanoseconds/1e9-self.odom_source_stamp) if self.odom_source_stamp is not None else math.inf,velocity=self.velocity,scan_world=list(self.scan_world),
            path=list(self.path),nodes=list(self.nodes),graph_ready=self.graph_ready,initial_ready=self.initial_ready,nav_ready=self.nav_ready,
            software_stop=self.software_stop,stop_cause=dict(self.stop_cause) if self.stop_cause else None,gui_age_s=now-self.gui_at if self.gui_at is not None else None,control_mode=self.control_mode,motion_recovery_required=self.motion_recovery_required,stop_reset_pending=bool(self.stop_reset_pending),navigation_busy=self.busy or self.server_active,busy=self.busy or self.server_active or self.return_active(),manual=self.manual,manual_stop_pending=self.manual_stop_pending,
            status=self.status,distance=self.distance,goal_status=self.goal_status,goal_target=self.goal_target,
            cancel_requested=self.cancel_requested,docking_hold=self.docking_hold,
            stationary_s=self.stop_observation.duration(self.goal_finished_at),stationary_samples=self.stop_observation.samples,
            power=self.power.snapshot(now),power48=self.power48.snapshot(now),manual_target=self.manual_target)

    def snapshot(self):
        with self.lock:
            taken=time.monotonic();state=self._snapshot(taken)
            state['snapshot_taken_at']=taken
            state['snapshot_completed_at']=time.monotonic()
            return state

    def close(self):
        self.emergency_stop(reason='panel_close',detail='用户关闭面板')
        self.stop_return('面板关闭')
        self.disable_manual('面板关闭')
        self.manual_tick()
        self.ros_executor.shutdown(timeout_sec=2);self.thread.join(timeout=2)
        self.destroy_node();rclpy.shutdown()

    def on_raw_odom(self,message):
        import json
        from raw_odom_mode import diagnostic
        with self.lock:
            try:
                data=json.loads(message.data)
                if data.get('error'):raise ValueError(data['error'])
                if self.raw_tracker.identity is None:return
                if data['relay_instance']!=self.raw_tracker.identity.get('relay_instance'):raise ValueError('原始封装会话改变')
                self.raw_tracker.update(data['pose'],data['velocity'],data['source_stamp'],time.monotonic(),
                    self.get_clock().now().nanoseconds/1e9,data['frame'],data['child'],data['publisher_gid'])
                if self.return_runtime is not None:self.event('return_raw_feedback',sample=self.raw_tracker.latest,relay_sequence=data.get('relay_sequence'),relay_instance=data.get('relay_instance'),relay_received_ros=data.get('relay_received_ros'))
            except (ValueError,KeyError,TypeError) as e:
                first=self.raw_tracker.error is None or self.raw_tracker.rejection is None
                if self.raw_tracker.rejection is None:self.raw_tracker.rejection={'reason':str(e),'envelope':diagnostic(locals().get('data')), 'raw_epoch':self.raw_tracker.epoch}
                self.raw_tracker.error=str(e)
                if first or not getattr(self,'raw_rejection_reported',None)==self.raw_tracker.epoch:
                    self.event('raw_odometry_rejected',rejection=self.raw_tracker.rejection);self.raw_rejection_reported=self.raw_tracker.epoch
                if self.return_active():self.latch_software_stop('raw_odometry_rejected',str(e));self.return_state=self.return_runtime.stop(str(e))

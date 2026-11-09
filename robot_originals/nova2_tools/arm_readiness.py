"""Read-only readiness: source-valid joints must match fresh MoveIt scene queries."""
import argparse,json,math,time,os
from pathlib import Path
import xml.etree.ElementTree as ET
JOINTS=tuple('joint'+str(i) for i in range(1,7))
def finite(v):return type(v) in (int,float) and math.isfinite(v)
def joint_values(names,positions):
    if len(names)!=len(positions) or len(set(names))!=len(names):raise ValueError('invalid joint names/positions')
    values=dict(zip(names,positions))
    if any(k not in values or not finite(values[k]) for k in JOINTS):raise ValueError('missing/nonfinite arm joint')
    return {k:values[k] for k in JOINTS}
def ready(state,now,joint_publishers,feedback_publishers,feedback_only=False,pose_ready=False,ik_ready=False):
    s=state.get('status',{});rx=s.get('source_progress_monotonic_ns');received=state.get('received')
    fresh=(s.get('valid') is True and type(rx) is int and finite(received)
           and 0<=now-rx/1e9<=.4 and 0<=now-received<=.3)
    return bool(fresh and joint_publishers==1 and feedback_publishers==1
                and (feedback_only or (state.get('model') is True and pose_ready and ik_ready)))
class Alignment:
    def __init__(self):self.count=0;self.last_tick=None;self.session=None;self.fault=None;self.reason='waiting for MoveIt real state'
    def accept(self,feedback,joints,joints_received,scene,query_started,query_finished,request_session,now):
        if self.fault:return False
        try:
            session=feedback['source_session'];tick=feedback['controller_timer'];rx=feedback['source_progress_monotonic_ns']
            if not isinstance(session,str) or not session or type(tick) is not int or type(rx) is not int:raise ValueError('invalid source identity')
            if self.session is not None and session!=self.session:self.fault='source session changed; restart readiness after inspection';raise ValueError(self.fault)
            if self.last_tick is not None and tick<self.last_tick:self.fault='source timer regressed';raise ValueError(self.fault)
            self.session=session
            if feedback.get('valid') is not True or request_session!=session:raise ValueError('source invalid or changed during query')
            if any(not finite(v) for v in (now,joints_received,query_started,query_finished)):raise ValueError('invalid check clock')
            if not (0<=now-rx/1e9<=.4 and 0<=now-joints_received<=.3 and query_started<=query_finished<=now and now-query_started<=.4):raise ValueError('feedback/joints/MoveIt query stale')
            if any(k not in joints or k not in scene or not finite(joints[k]) or not finite(scene[k]) for k in JOINTS):raise ValueError('MoveIt joints incomplete/nonfinite')
            error=max(abs(math.degrees(scene[k]-joints[k])) for k in JOINTS)
            if error>.5:raise ValueError('MoveIt internal joints differ %.6f deg'%error)
            if tick==self.last_tick:return False
            self.last_tick=tick;self.count+=1;self.reason=None
            return self.count>=3
        except (KeyError,ValueError,TypeError) as error:self.count=0;self.reason=str(error);return False

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from std_msgs.msg import String
    from sensor_msgs.msg import JointState
    from dobot_msgs_v4.srv import GetPose
    from moveit_msgs.srv import GetPositionIK,GetPlanningScene
    from moveit_msgs.msg import PlanningSceneComponents
    parser=argparse.ArgumentParser();parser.add_argument('--feedback-only',action='store_true');parser.add_argument('--timeout',type=float,default=45.);args=parser.parse_args()
    if not 0<args.timeout<=45:parser.error('timeout must be positive and no more than 45s')
    rclpy.init();n=Node('nova2_startup_readiness');state={};alignment=Alignment()
    def status(msg):
        try:
            d=json.loads(msg.data)
            if not isinstance(d,dict):raise ValueError('invalid status')
            state['status']=d;state['received']=time.monotonic()
        except ValueError:state.clear()
    def description(msg):
        try:
            model=ET.fromstring(msg.data);state['model']=model.get('name')=='nova2_robot' and all(model.find("joint[@name='%s']"%k) is not None for k in JOINTS)
        except ET.ParseError:state['model']=False
    def joint(msg):
        try:state['joints']=joint_values(msg.name,msg.position);state['joints_received']=time.monotonic()
        except ValueError:state.pop('joints',None)
    n.create_subscription(String,'/nova2/feedback_status',status,10)
    n.create_subscription(JointState,'/joint_states',joint,10)
    n.create_subscription(String,'/robot_description',description,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pose=n.create_client(GetPose,'/dobot_bringup_ros2/srv/GetPose');ik=n.create_client(GetPositionIK,'/compute_ik');scene=n.create_client(GetPlanningScene,'/get_planning_scene')
    deadline=time.monotonic()+args.timeout;good=0;pending=None;started=None;request_session=None
    try:
        while time.monotonic()<deadline:
            if os.environ.get('CHARGE_ROBOT_SEQUENCE_CANCEL_FILE') and Path(os.environ['CHARGE_ROBOT_SEQUENCE_CANCEL_FILE']).exists():raise RuntimeError('sequence cancelled; no further startup')
            rclpy.spin_once(n,timeout_sec=.05);now=time.monotonic()
            available=ready(state,now,n.count_publishers('/joint_states'),n.count_publishers('/dobot_bringup_ros2/msg/FeedInfo'),args.feedback_only,pose.service_is_ready(),ik.service_is_ready())
            if args.feedback_only:
                good=good+1 if available else 0
                if good>=3:print('Nova2 source feedback ready');return 0
                continue
            if not available or not scene.service_is_ready():
                alignment.count=0
                if pending:pending.cancel();pending=None
                continue
            if pending and now-started>.4:pending.cancel();pending=None;alignment.count=0;alignment.reason='MoveIt scene query timeout';continue
            if pending and pending.done():
                try:
                    result=pending.result();j=result.scene.robot_state.joint_state
                    if alignment.accept(state['status'],state.get('joints',{}),state.get('joints_received',0),joint_values(j.name,j.position),started,now,request_session,now):
                        print('Nova2 ready: 3 advancing source samples match current MoveIt internal state');return 0
                except (ValueError,AttributeError,RuntimeError) as error:alignment.count=0;alignment.reason=str(error)
                pending=None
            if pending is None:
                req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.ROBOT_STATE
                started=time.monotonic();request_session=state['status'].get('source_session');pending=scene.call_async(req)
        print('Nova2 readiness refused: '+str(alignment.fault or alignment.reason));return 2
    finally:
        if pending:pending.cancel()
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())

#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, RobotState
from moveit_msgs.srv import GetCartesianPath
from shape_msgs.msg import SolidPrimitive

def qmul(a,b):
    ax,ay,az,aw=a; bx,by,bz,bw=b
    return (aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz)
def qrot(q,v):
    return tuple(x*2 for x in (q[1]*(q[2]*v[0]-q[0]*v[2])-q[2]*(q[0]*v[1]-q[1]*v[0]), q[2]*(q[0]*v[2]-q[2]*v[1])-q[0]*(q[1]*v[2]-q[2]*v[0]), q[0]*(q[1]*v[2]-q[2]*v[1])-q[1]*(q[0]*v[1]-q[1]*v[0]))) if False else None
def q_from_rpy(r,p,y):
    cr,sr=math.cos(r/2),math.sin(r/2); cp,sp=math.cos(p/2),math.sin(p/2); cy,sy=math.cos(y/2),math.sin(y/2)
    return (sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy+sr*sp*cy,cr*cp*cy+sr*sp*sy)
def rotate(q,v):
    x,y,z,w=q; return (v[0]*(1-2*y*y-2*z*z)+v[1]*(2*x*y-2*z*w)+v[2]*(2*x*z+2*y*w), v[0]*(2*x*y+2*z*w)+v[1]*(1-2*x*x-2*z*z)+v[2]*(2*y*z-2*x*w), v[0]*(2*x*z-2*y*w)+v[1]*(2*y*z+2*x*w)+v[2]*(1-2*x*x-2*y*y))
def pose_for(port,q_port,offset,q_rel):
    q=qmul(q_port,q_rel); d=rotate(q_port,offset); p=Pose(); p.position.x=port[0]+d[0]; p.position.y=port[1]+d[1]; p.position.z=port[2]+d[2]; p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q; return p
def constraints(p,frame,link):
    c=Constraints(); pc=PositionConstraint(); pc.header.frame_id=frame; pc.link_name=link; s=SolidPrimitive(); s.type=SolidPrimitive.SPHERE; s.dimensions=[0.004]; pc.constraint_region.primitives=[s]; pc.constraint_region.primitive_poses=[p]; pc.weight=1.0; c.position_constraints=[pc]
    oc=OrientationConstraint(); oc.header.frame_id=frame; oc.link_name=link; oc.orientation=p.orientation; oc.absolute_x_axis_tolerance=0.03; oc.absolute_y_axis_tolerance=0.03; oc.absolute_z_axis_tolerance=0.03; oc.weight=1.0; c.orientation_constraints=[oc]; return c
class AutoDock(Node):
    def __init__(self):
        super().__init__('auto_dock'); self.group=self.declare_parameter('planning_group','nova2_group').value; self.link=self.declare_parameter('end_effector_link','Link6').value; self.frame=self.declare_parameter('planning_frame','dummy_link').value; self.port=list(self.declare_parameter('port_xyz',[0.4,0.0,0.45]).value); self.depth=float(self.declare_parameter('insert_depth',.035).value); self.pre=float(self.declare_parameter('pre_insert_distance',.12).value); self.qrel=q_from_rpy(*list(self.declare_parameter('dock_relative_rpy',[1.5708,0.0,1.5708]).value)); self.move=ActionClient(self,MoveGroup,'/move_action'); self.exec=ActionClient(self,ExecuteTrajectory,'/execute_trajectory'); self.cart=self.create_client(GetCartesianPath,'/compute_cartesian_path'); self.sub=self.create_subscription(PoseStamped,'/charging_port_target',self.target_cb,10)
    def target_cb(self,m):
        if m.header.frame_id==self.frame: self.port=[m.pose.position.x,m.pose.position.y,m.pose.position.z]
    def run(self):
        qport=(0,0,0,1); pre=pose_for(self.port,qport,(-self.pre,0,0),self.qrel); dock=pose_for(self.port,qport,(self.depth,0,0),self.qrel); self.get_logger().info('AUTO_DOCK: planning approach')
        if not self.move.wait_for_server(10): return self.fail('MoveGroup unavailable')
        g=MoveGroup.Goal(); g.request.group_name=self.group; g.request.goal_constraints=[constraints(pre,self.frame,self.link)]; g.request.allowed_planning_time=5.; g.request.num_planning_attempts=5; g.request.max_velocity_scaling_factor=.15; g.request.max_acceleration_scaling_factor=.15; g.planning_options.plan_only=True
        f=self.move.send_goal_async(g); rclpy.spin_until_future_complete(self, f, timeout_sec=15); h=f.result()
        if h is None or not h.accepted:return self.fail('approach goal rejected')
        rf=h.get_result_async(); rclpy.spin_until_future_complete(self, rf, timeout_sec=20); res=rf.result().result
        if res.error_code.val!=1:return self.fail(f'approach planning failed: {res.error_code.val}')
        self.get_logger().info('APPROACH: planned')
        if not self.execute(res.planned_trajectory):return self.fail('approach execution failed')
        self.get_logger().info('APPROACH: executed; planning straight insertion')
        req=GetCartesianPath.Request(); req.header.frame_id=self.frame; req.group_name=self.group; req.link_name=self.link; req.waypoints=[pre,dock]; req.max_step=.003; req.jump_threshold=0.; req.avoid_collisions=True
        # The robot has already executed the approach. Use the final point of
        # that trajectory as the Cartesian insertion start state.
        req.start_state = RobotState()
        jt = res.planned_trajectory.joint_trajectory
        if jt.points:
            req.start_state.joint_state.name = list(jt.joint_names)
            req.start_state.joint_state.position = list(jt.points[-1].positions)
        if not self.cart.wait_for_service(10):return self.fail('Cartesian service unavailable')
        cf=self.cart.call_async(req); rclpy.spin_until_future_complete(self, cf, timeout_sec=20); cr=cf.result()
        if cr is None or cr.fraction<.98:return self.fail(f'insert path incomplete: {0 if cr is None else cr.fraction:.3f}')
        if not self.execute(cr.solution):return self.fail('insert execution failed')
        self.get_logger().info('INSERT: executed; DOCKED: SUCCESS')
    def execute(self,traj):
        if not self.exec.wait_for_server(10):return False
        g=ExecuteTrajectory.Goal(); g.trajectory=traj; f=self.exec.send_goal_async(g); rclpy.spin_until_future_complete(self, f, timeout_sec=20); h=f.result()
        if h is None or not h.accepted:return False
        rf=h.get_result_async(); rclpy.spin_until_future_complete(self, rf, timeout_sec=30); return rf.result() is not None and rf.result().result.error_code.val==1
    def fail(self,msg): self.get_logger().error('AUTO_DOCK FAILED: '+msg); return False
def main():
    rclpy.init()
    n=AutoDock()
    # Run the action sequence from the main thread. Blocking waits inside a
    # single-threaded timer callback can prevent Humble action futures from
    # completing even though MoveIt computed the plan.
    rclpy.spin_once(n, timeout_sec=2.0)
    n.run()
    n.destroy_node()
    rclpy.shutdown()

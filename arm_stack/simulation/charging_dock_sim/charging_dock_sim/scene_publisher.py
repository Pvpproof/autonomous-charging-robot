#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def q_from_rpy(r,p,y):
    cr,sr=math.cos(r/2),math.sin(r/2); cp,sp=math.cos(p/2),math.sin(p/2); cy,sy=math.cos(y/2),math.sin(y/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy+sr*sp*cy, cr*cp*cy+sr*sp*sy)

class Scene(Node):
    def __init__(self):
        super().__init__('charging_dock_scene')
        self.declare_parameter('planning_frame','dummy_link')
        self.declare_parameter('port_frame','charging_port_entry')
        self.declare_parameter('target_topic','/charging_port_target')
        self.declare_parameter('port_xyz',[0.40,0.0,0.45])
        self.declare_parameter('port_rpy',[0.0,0.0,0.0])
        self.declare_parameter('port_size',[0.075,0.075])
        self.declare_parameter('port_wall',0.006)
        self.declare_parameter('port_depth',0.035)
        self.declare_parameter('gun_size',[0.050,0.040])
        self.declare_parameter('gun_insert_length',0.035)
        self.frame=self.get_parameter('planning_frame').value
        self.port_frame=self.get_parameter('port_frame').value
        self.port=list(self.get_parameter('port_xyz').value)
        self.rpy=list(self.get_parameter('port_rpy').value)
        self.size=list(self.get_parameter('port_size').value)
        self.wall=float(self.get_parameter('port_wall').value)
        self.depth=float(self.get_parameter('port_depth').value)
        self.gun_size=list(self.get_parameter('gun_size').value)
        self.gun_len=float(self.get_parameter('gun_insert_length').value)
        self.obj_pub=self.create_publisher(CollisionObject,'/collision_object',10)
        self.att_pub=self.create_publisher(AttachedCollisionObject,'/attached_collision_object',10)
        self.marker_pub=self.create_publisher(MarkerArray,'/charging_dock_markers',10)
        self.target_pub=self.create_publisher(PoseStamped,self.get_parameter('target_topic').value,10)
        self.target_sub=self.create_subscription(PoseStamped,self.get_parameter('target_topic').value,self.target_cb,10)
        self.tf=TransformBroadcaster(self)
        self.last_signature=None
        self.timer=self.create_timer(1.0,self.publish_all)
        self.publish_all()
    def target_cb(self,msg):
        if msg.header.frame_id and msg.header.frame_id != self.frame:
            self.get_logger().warn(f'Ignoring target in frame {msg.header.frame_id}; expected {self.frame}')
            return
        self.port=[msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
        q=msg.pose.orientation
        # Convert quaternion to RPY only for visualization-free internal storage.
        self.q_port=(q.x,q.y,q.z,q.w)
        self.publish_all()
    def box(self,oid,xyz,size,frame=None):
        o=CollisionObject(); o.header.frame_id=frame or self.frame; o.id=oid
        s=SolidPrimitive(); s.type=SolidPrimitive.BOX; s.dimensions=[float(v) for v in size]
        p=Pose(); p.position.x,p.position.y,p.position.z=xyz; p.orientation.w=1.0
        o.primitives=[s]; o.primitive_poses=[p]; o.operation=CollisionObject.ADD
        return o
    def publish_all(self):
        x,y,z=self.port; w,h=self.size; t=self.wall
        # Port frame is identity-oriented in this first movable-target version.
        objs=[
          self.box('port_top',[x,y,z+h/2-t/2],[self.depth,w,t]),
          self.box('port_bottom',[x,y,z-h/2+t/2],[self.depth,w,t]),
          self.box('port_left',[x,y-w/2+t/2,z],[self.depth,t,h-2*t]),
          self.box('port_right',[x,y+w/2-t/2,z],[self.depth,t,h-2*t]),
        ]
        # Replace the four port pieces atomically enough for a simple demo.
        for o in objs: self.obj_pub.publish(o)
        gun=AttachedCollisionObject(); gun.link_name='Link6'; gun.object.header.frame_id='Link6'; gun.object.id='charging_gun'
        s=SolidPrimitive(); s.type=SolidPrimitive.BOX; s.dimensions=[self.gun_len,self.gun_size[0],self.gun_size[1]]
        p=Pose(); p.position.x=-self.gun_len/2; p.orientation.w=1.0
        gun.object.primitives=[s]; gun.object.primitive_poses=[p]; gun.object.operation=CollisionObject.ADD
        gun.touch_links=['Link6']; self.att_pub.publish(gun)
        ts=self.get_clock().now().to_msg()
        for child,dx in [(self.port_frame,0.0),('charging_port_pre_insert',-0.12),('charging_port_insert',self.depth)]:
            tr=TransformStamped(); tr.header.stamp=ts; tr.header.frame_id=self.frame; tr.child_frame_id=child
            tr.transform.translation.x=x+dx; tr.transform.translation.y=y; tr.transform.translation.z=z
            tr.transform.rotation.w=1.0; self.tf.sendTransform(tr)
        m=MarkerArray()
        for i,(name,px,col) in enumerate([('port_center',x,(0.1,0.8,1.0,1.0)),('pre_insert',x-0.12,(1.0,0.8,0.1,1.0)),('dock',x+self.depth,(0.1,1.0,0.1,1.0))]):
            a=Marker(); a.header.frame_id=self.frame; a.header.stamp=ts; a.ns='charging_dock'; a.id=i; a.type=Marker.SPHERE; a.action=Marker.ADD
            a.pose.position.x,a.pose.position.y,a.pose.position.z=px,y,z; a.pose.orientation.w=1.0; a.scale.x=a.scale.y=a.scale.z=0.018; a.color.r,a.color.g,a.color.b,a.color.a=col; m.markers.append(a)
        self.marker_pub.publish(m)
        target=PoseStamped(); target.header.frame_id=self.frame; target.header.stamp=ts; target.pose.position.x=x; target.pose.position.y=y; target.pose.position.z=z; target.pose.orientation.w=1.0
        self.target_pub.publish(target)

def main():
    rclpy.init(); n=Scene()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    n.destroy_node(); rclpy.shutdown()

"""Source-progress gate. Importing this module does not start ROS or motion."""
import json, math, time
from source_feedback import SourceProgress

class Gate:
    def __init__(self,clock=time.monotonic):
        self.clock=clock;self.progress=SourceProgress(clock);self.pose=None;self.good=0;self.reason='waiting for controller source';self.latched=False
    def accept(self,data):
        try:
            if self.latched:raise RuntimeError(self.reason)
            if not isinstance(data,dict):raise RuntimeError('FeedInfo is not an object')
            q=data.get('q_actual')
            if not isinstance(q,list) or len(q)!=6 or any(type(x) not in (int,float) or not math.isfinite(x) for x in q):raise RuntimeError('invalid q_actual')
            if not self.progress.accept(data):
                self.progress.check();return None
            self.progress.check() if self.progress.count>=2 else None
            self.good+=1
            if self.good<3:return None
            self.pose=[math.radians(x) for x in q];self.reason=None
            return list(self.pose)
        except (RuntimeError,ValueError,TypeError) as error:
            self.reason=str(error);self.pose=None;self.good=0
            if self.progress.fault:self.latched=True
            return None
    def status(self):
        try:self.progress.check()
        except RuntimeError as error:self.pose=None;self.good=0;self.reason=str(error)
        return dict(valid=self.pose is not None and not self.latched,reason=self.reason,latched=self.latched,
                    source_session=self.progress.session,controller_timer=self.progress.tick,
                    source_progress_monotonic_ns=self.progress.rx_ns,checked_at=self.clock())

def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import JointState
    rclpy.init();node=Node('nova2_verified_joint_feedback');gate=Gate()
    pub=node.create_publisher(JointState,'/joint_states',10)
    status=node.create_publisher(String,'/nova2/feedback_status',10)
    def receive(message):
        try:data=json.loads(message.data)
        except (ValueError,TypeError):gate.good=0;gate.pose=None;gate.reason='invalid FeedInfo JSON';return
        q=gate.accept(data)
        if q is not None:
            msg=JointState();msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id='dummy_link'
            msg.name=['joint'+str(i) for i in range(1,7)];msg.position=q;pub.publish(msg)
    node.create_subscription(String,'/dobot_bringup_ros2/msg/FeedInfo',receive,10)
    def tick():
        msg=String();msg.data=json.dumps(gate.status());status.publish(msg)
    node.create_timer(.1,tick)
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()

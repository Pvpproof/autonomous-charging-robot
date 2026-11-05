"""Independent ROS command arbiter. Defaults to an isolated test namespace."""
import argparse
import math
import json
from pathlib import Path
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import String
from control_arbiter import Arbiter
from release_policy import ReleasePolicy,TrialLimiter
from runtime_provenance import process_identity,boot_id
from driver_watchdog_bridge import DriverWatchdogControl
from raw_odom_mode import RawTracker,SOURCE
from rcl_interfaces.srv import GetParameters

ROOT = Path(__file__).resolve().parent


class ArbiterNode(DriverWatchdogControl,Node):
    def __init__(self, *, isolated=True, context=None, test_profile="supervised_route", test_return_speed=.05, test_config=None,test_driver_watchdog=False):
        self.prefix = '/charging_return_isolated' if isolated else '/charging_control'
        self.isolated = isolated
        super().__init__('charging_control_arbiter', context=context,
                         start_parameter_services=False, enable_rosout=False)
        if self.resolve_topic_name(self.prefix+'/chassis')!=self.prefix+'/chassis':
            self.destroy_node();raise RuntimeError('chassis output remapping is forbidden')
        self.core = Arbiter();self.instance_id=uuid.uuid4().hex
        if isolated:config=test_config or {'profile':test_profile,'return_speed_m_s':test_return_speed}
        else:
            try:config=json.loads((ROOT/'config/return_commissioning.json').read_text())
            except (OSError,ValueError):config={'profile':'disabled'}
        self.policy=ReleasePolicy(config,isolated_test=isolated)
        self.limiter=TrialLimiter(self.policy);self.core.return_speed=self.policy.return_speed;self.output_sequence=0
        self.commissioned=self.policy.enabled
        if self.policy.profile=='supervised_trial' and self.policy.enabled:
            def claim_trial(key,packet,now):
                # Atomic durable permit consumption before the first nonzero output.
                import os
                directory=Path('/tmp/return-work/trial-ledger') if self.isolated else ROOT/'logs'/'full-trial-authorizations';directory.mkdir(parents=True,exist_ok=True)
                name=directory/(self.policy.trial_authorization['authorization_id']+'.json')
                try:fd=os.open(name,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
                except FileExistsError:self.limiter.full_trial_consumed=True;return False
                with os.fdopen(fd,'w') as f:
                    json.dump(dict(key=key,packet=packet,claimed_monotonic_s=now,arbiter_instance=self.instance_id,source_key=self.raw_tracker.source_key),f);f.flush();os.fsync(f.fileno())
                directory_fd=os.open(directory,os.O_RDONLY|os.O_DIRECTORY)
                try:os.fsync(directory_fd)
                finally:os.close(directory_fd)
                return True
            self.limiter.claim_trial=claim_trial
            ledger=Path('/tmp/return-work/trial-ledger') if isolated else ROOT/'logs'/'full-trial-authorizations'
            self.limiter.full_trial_consumed=(ledger/(self.policy.trial_authorization['authorization_id']+'.json')).exists()
        self.init_driver_watchdog(not isolated or test_driver_watchdog)
        self.position=None;self.measured_velocity=(0.,0.)
        self.raw_tracker=RawTracker();self.raw_source_identity=None;self.raw_gid=None;self.relay_instance=None
        self.driver_gate_verified=isolated;self.driver_gate_at=time.monotonic() if isolated else None
        self.driver_process='isolated-driver' if isolated else None
        self.localizer_process='isolated-localizer' if isolated else None
        self.driver_gid=None;self.localizer_gid=None
        self.gate_client=None;self.gate_future=None
        if not isolated:
            self.gate_client=self.create_client(GetParameters,'/ychd_robot/get_parameters')
            self.create_timer(.5,self.check_driver_gate)
        self.pub = self.create_publisher(Twist, self.prefix+'/chassis', 1)
        self.status = self.create_publisher(String, self.prefix+'/status', 1)
        self.create_subscription(String, self.prefix+'/authority', self.authority, 1)
        self.create_subscription(String,self.prefix+'/raw_odometry',self.raw_odom,10)
        self.create_subscription(Twist, self.prefix+'/navigation', self.navigation, 1)
        self.create_subscription(Odometry, self.prefix+'/odom' if isolated else '/odom',
                                 self.odom, qos_profile_sensor_data)
        self.create_subscription(GoalStatusArray, self.prefix+'/nav_status' if isolated else
                                 '/navigate_to_pose/_action/status', self.nav_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.05, self.tick)

    def authority(self, msg):
        try:
            packet=json.loads(msg.data)
            if not isinstance(packet,dict):self.core.fail('invalid authority object');return
            if packet.get('instance_id')!=self.instance_id:
                self.core.fail('arbiter instance changed or missing');return
            if packet.get('mode')=='return':
                if packet.get('coordinate_mode')!=SOURCE:
                    self.core.fail('raw odom return mode required');return
                if packet.get('raw_source_key')!=self.raw_tracker.source_key or packet.get('raw_epoch')!=self.raw_tracker.epoch or self.raw_tracker.error:
                    self.core.fail('raw odom identity/epoch invalid');return
            if self.driver_watchdog_required and self.handle_driver_reset(packet):return
            self.core.receive(packet,time.monotonic(),self.get_clock().now().nanoseconds/1e9)
            if packet.get('reset') is True and self.core.fault is None:
                self.limiter.reset(quiet=self.core.quiet(time.monotonic()),explicit_reset=True)
                if self.raw_tracker.error:self.raw_tracker.reset()
        except (ValueError, TypeError):
            self.core.fail('invalid authority JSON')

    def navigation(self, msg):
        self.core.navigation(msg.linear.x, msg.angular.z, time.monotonic())

    def odom(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        age = self.get_clock().now().nanoseconds/1e9-stamp
        valid = (msg.header.frame_id == 'odom_combined' and msg.child_frame_id == 'base_footprint'
                 and 0 <= age <= .25)
        self.position=(msg.pose.pose.position.x,msg.pose.pose.position.y)
        self.measured_velocity=(msg.twist.twist.linear.x,msg.twist.twist.angular.z)
        self.core.odom(msg.twist.twist.linear.x, msg.twist.twist.angular.z,
                       stamp, time.monotonic(), valid)
    def raw_odom(self,message):
        try:
            data=json.loads(message.data)
            if data.get('error'):raise ValueError(data['error'])
            if self.raw_gid is None:return
            if data['publisher_gid']!=self.raw_gid:raise ValueError('raw envelope source GID mismatch')
            if self.relay_instance is not None and self.relay_instance!=data['relay_instance']:
                self.raw_tracker.error='raw relay restarted';self.core.fail('raw relay restarted')
            self.relay_instance=data['relay_instance']
            if self.raw_tracker.identity is None:return
            if self.relay_instance!=self.raw_tracker.identity.get('relay_instance'):return
            self.raw_tracker.update(data['pose'],data['velocity'],data['source_stamp'],time.monotonic(),
                self.get_clock().now().nanoseconds/1e9,data['frame'],data['child'],data['publisher_gid'])
        except (ValueError,KeyError,TypeError) as e:self.raw_tracker.error=str(e);self.core.fail(str(e))

    def nav_status(self, msg):
        self.core.nav_active = any(item.status in (1, 2, 3) for item in msg.status_list)

    def graph_ok(self):
        p = self.prefix
        self.graph_ready=False
        def reject(reason):
            self.graph_reason=reason
            return False
        def identities(items):
            return [(x.node_namespace, x.node_name) for x in items]
        pubs=identities(self.get_publishers_info_by_topic(p+'/chassis'))
        if pubs != [('/', self.get_name())]:
            return reject('chassis publishers: '+repr(pubs))
        expected_sink = 'charging_return_test_sink' if self.isolated else 'ychd_robot'
        sinks=self.get_subscriptions_info_by_topic(p+'/chassis')
        if identities(sinks) != [('/', expected_sink)]:return reject('chassis subscribers: '+repr(identities(sinks)))
        self.driver_gid=bytes(sinks[0].endpoint_gid).hex()
        raw_topic=p+'/odom' if self.isolated else '/odom';raw_pubs=self.get_publishers_info_by_topic(raw_topic)
        expected_raw='controller_server' if self.isolated else 'ychd_robot'
        if identities(raw_pubs)!=[('/',expected_raw)]:return reject('raw odom publishers: '+repr(identities(raw_pubs)))
        self.raw_gid=bytes(raw_pubs[0].endpoint_gid)[:16].hex()
        relays=identities(self.get_publishers_info_by_topic(p+'/raw_odometry'))
        if relays!=[('/', 'charging_raw_odom_relay')]:return reject('raw envelope publishers: '+repr(relays))
        if self.driver_process and self.driver_gate_verified and self.relay_instance:
            self.raw_source_identity=dict(boot_id='isolated' if self.isolated else boot_id(),driver=self.driver_process,publisher_gid=self.raw_gid,relay_instance=self.relay_instance,topic=raw_topic,source_kind=SOURCE)
            self.raw_tracker.bind(self.raw_source_identity)
        if not self.isolated:
            if not self.driver_gate_verified:return reject('driver gate parameter not yet verified or rejected')
            if self.driver_gate_at is None or time.monotonic()-self.driver_gate_at>1.5:return reject('driver gate verification expired')
            if not self.driver_process:return reject('driver process identity missing or ambiguous')
            inputs=self.get_subscriber_names_and_types_by_node('ychd_robot','/')
            services=self.get_service_names_and_types_by_node('ychd_robot','/')
            bypass=[name for name,_ in inputs if name in ('/red_vel','/robot_recharge_flag')]+[name for name,_ in services if name=='/set_charge']
            if bypass:return reject('driver bypass interfaces: '+repr(bypass))
            amcl=[x for x in self.get_publishers_info_by_topic('/tf') if x.node_name=='amcl' and x.node_namespace=='/']
            if self.core.mode=='navigation' and (not self.localizer_process or len(amcl)!=1):return reject('navigation localizer identity or AMCL TF not unique')
            self.localizer_gid=bytes(amcl[0].endpoint_gid).hex() if len(amcl)==1 else None
        owners=identities(self.get_publishers_info_by_topic(p+'/authority'))
        if owners != [('/', 'charging_navigation_panel')]:return reject('authority publishers: '+repr(owners))
        sources = identities(self.get_publishers_info_by_topic(p+'/navigation'))
        limits={('/', 'controller_server'):1,('/', 'behavior_server'):4}
        counts={}
        for source in sources:counts[source]=counts.get(source,0)+1
        if any(source not in limits or count>limits.get(source,0) for source,count in counts.items()):return reject('navigation publishers: '+repr(counts))
        if not self.isolated and self.get_publishers_info_by_topic('/cmd_vel'):return reject('legacy /cmd_vel bypass publisher present')
        if self.driver_watchdog_required and not self.driver_watchdog_graph_ok():return reject('driver watchdog state/service graph invalid')
        if self.driver_watchdog_required and not self.driver_watchdog.contract_ready(time.monotonic()):return reject('driver watchdog v2 state missing or stale')
        self.graph_ready=True;self.graph_reason=None
        return True

    def check_driver_gate(self):
        self.driver_process=process_identity('driver');self.localizer_process=process_identity('localizer')
        if self.gate_future is not None and not self.gate_future.done():return
        if not self.gate_client.service_is_ready():
            self.driver_gate_verified=False;return
        request=GetParameters.Request();request.names=['charging_return_driver_gate_version']
        self.gate_future=self.gate_client.call_async(request)
        self.gate_future.add_done_callback(self.gate_response)

    def gate_response(self,future):
        try:
            values=future.result().values
            self.driver_gate_verified=len(values)==1 and values[0].type==2 and values[0].integer_value==2
            self.driver_gate_at=time.monotonic()
        except Exception:self.driver_gate_verified=False

    def tick(self):
        now = time.monotonic()
        self.poll_driver_reset(now)
        v, w = self.core.tick(now, graph_ok=self.graph_ok())
        if self.driver_watchdog_required and not self.driver_watchdog.motion_ready(now):
            v,w=0.,0.
            if self.core.fault is None:self.core.fail('driver watchdog locked or stale: '+str((self.driver_watchdog.state or {}).get('reason')))
        if self.core.mode=='return':
            try:self.raw_tracker.current(now,self.get_clock().now().nanoseconds/1e9)
            except ValueError as e:v,w=self.core.fail(str(e))
        v,w=self.limiter.filter(self.core.mode,self.core.packet,(v,w),now,
            odom_pose=self.position,odom_stamp=self.core.odom_stamp,
            odom_age=now-self.core.odom_at if self.core.odom_at is not None else float('inf'),
            measured_velocity=self.measured_velocity)
        if self.limiter.fault:self.core.fail(self.limiter.fault)
        msg = Twist(); msg.linear.x = float(v); msg.angular.z = float(w)
        self.pub.publish(msg)
        self.output_sequence+=1;published_at=time.monotonic();published_ros=self.get_clock().now().nanoseconds/1e9
        state = String()
        state.data = json.dumps(dict(owner=self.core.owner, sequence=self.core.sequence,
            mode=self.core.mode, fault=self.core.fault, commissioned=self.commissioned,
            return_start_mode=self.policy.start_mode,return_speed_m_s=self.policy.return_speed,feedback_speed_limit_m_s=self.policy.feedback_speed_limit,full_trial_max_route_m=self.policy.trial_max_route_m,full_trial_max_time_s=self.policy.trial_max_time_s,full_trial_consumed=self.limiter.full_trial_consumed,output_sequence=self.output_sequence,output_monotonic_s=published_at,output_ros_stamp=published_ros,
            output_v_m_s=float(v),output_w_rad_s=float(w),authority_sequence=self.core.sequence,
            run_session=(self.core.packet or {}).get("run_session"),run_sequence=(self.core.packet or {}).get("run_sequence"),run_state=(self.core.packet or {}).get("run_state"),
            trial_id=(self.core.packet or {}).get("trial_id"),held=(self.core.packet or {}).get("held",False),
            instance_id=self.instance_id,release_profile=self.policy.profile,release_errors=self.policy.errors,
            graph_ready=self.graph_ready,graph_reason=self.graph_reason,driver_gate_verified=self.driver_gate_verified,driver_watchdog_required=self.driver_watchdog_required,
            driver_watchdog_state=self.driver_watchdog.state,driver_watchdog_ready=self.driver_watchdog.motion_ready(now),driver_reset_pending=self.driver_reset_pending is not None,
            driver_identity=dict(process=self.driver_process,sink_gid=self.driver_gid) if self.driver_process and self.driver_gid else None,
            localizer_identity=dict(process=self.localizer_process,tf_gid=self.localizer_gid) if self.localizer_process else None,
            raw_source_identity=self.raw_source_identity,raw_source_key=self.raw_tracker.source_key,raw_epoch=self.raw_tracker.epoch,
            raw_odom_ready=self.raw_tracker.latest is not None and self.raw_tracker.error is None and now-self.raw_tracker.latest['received_at']<=.25,raw_odom_error=self.raw_tracker.error,raw_odom_rejection=self.raw_tracker.rejection,
            independent_trial_distance_m=self.limiter.distance,hardware_validated=False,
            output_topic=self.prefix+'/chassis', detects_new_obstacles=False))
        self.status.publish(state)

    def destroy_node(self):
        if hasattr(self, 'pub'): self.pub.publish(Twist())
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--live', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = ArbiterNode(isolated=not args.live)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()

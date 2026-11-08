"""Explicit stationary reset: driver ACK, fresh driver state, arbiter ACK.
No recovery event starts motion. A cancelled GUI request cannot unlock the arbiter.
"""
import copy
import json
import time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from charging_robot.control.driver_watchdog_client import DriverWatchdogState

class DriverWatchdogControl:
    def init_driver_watchdog(self,required):
        self.driver_watchdog_required=required;self.driver_watchdog=DriverWatchdogState()
        self.driver_reset_pending=None;self.driver_reset_client=None
        if required:
            prefix=self.prefix+'/driver' if self.isolated else '/ychd_robot'
            self.driver_watchdog_prefix=prefix
            self.driver_reset_client=self.create_client(Trigger,prefix+'/reset_command_watchdog')
            self.create_subscription(String,prefix+'/command_watchdog_state',self.on_driver_watchdog,1)

    def driver_watchdog_graph_ok(self):
        expected='charging_watchdog_test' if self.isolated else 'ychd_robot'
        publishers=self.get_publishers_info_by_topic(self.driver_watchdog_prefix+'/command_watchdog_state')
        if [(p.node_namespace,p.node_name) for p in publishers]!=[('/',expected)]:return False
        services=self.get_service_names_and_types_by_node(expected,'/')
        return (self.driver_watchdog_prefix+'/reset_command_watchdog',['std_srvs/srv/Trigger']) in services

    def on_driver_watchdog(self,message):
        try:
            changed=self.driver_watchdog.observe(json.loads(message.data),time.monotonic())
            if changed:
                self.driver_reset_pending=None;self.core.fail('driver watchdog session changed; explicit reset required')
        except (ValueError,TypeError) as error:
            self.driver_watchdog.error=str(error);self.core.fail(str(error))

    def handle_driver_reset(self,packet):
        now=time.monotonic();ros_now=self.get_clock().now().nanoseconds/1e9
        pending=self.driver_reset_pending
        if packet.get('reset') is True:
            if pending is not None:self.core.fail('driver reset already pending');return True
            token=packet.get('reset_request_id')
            if not isinstance(token,str) or not token:self.core.fail('explicit driver reset identity missing');return True
            probe=copy.deepcopy(self.core);probe.receive(packet,now,ros_now)
            if probe.fault:self.core.fail(probe.fault);return True
            if not self.driver_watchdog.contract_ready(now):self.core.fail('driver watchdog state unavailable for reset');return True
            if not self.driver_reset_client.service_is_ready():self.core.fail('driver watchdog reset service unavailable');return True
            future=self.driver_reset_client.call_async(Trigger.Request())
            self.driver_reset_pending=dict(id=token,owner=packet['owner'],packet=dict(packet),requested_at=now,
                session=self.driver_watchdog.state['session'],future=future,ack=None)
            self.core.fail('waiting for explicit driver watchdog reset ACK')
            return True
        if pending is None:return False
        if (packet.get('reset_request_id')!=pending['id'] or packet.get('owner')!=pending['owner'] or
                packet.get('mode')!='stop' or packet.get('v')!=0 or packet.get('w')!=0):
            self.driver_reset_pending=None;self.core.fail('driver reset cancelled; no automatic unlock');return True
        if packet.get('sequence',-1)>pending['packet']['sequence']:
            candidate=dict(packet,reset=True)
            probe=copy.deepcopy(self.core);probe.receive(candidate,now,ros_now)
            if probe.fault:self.driver_reset_pending=None;self.core.fail(probe.fault);return True
            pending['packet']=candidate
        return True

    def poll_driver_reset(self,now):
        pending=self.driver_reset_pending
        if pending is None:return
        if not 0<=now-pending['requested_at']<=1.5:
            self.driver_reset_pending=None;self.core.fail('driver reset ACK timed out');return
        future=pending['future']
        if future.done() and pending['ack'] is None:
            try:
                response=future.result();ack=json.loads(response.message)
                DriverWatchdogState.validate(ack,now)
                if not response.success or ack['session']!=pending['session'] or ack['latched'] is not False:
                    raise ValueError('driver reset rejected or session changed')
                pending['ack']=ack
            except Exception as error:
                self.driver_reset_pending=None;self.core.fail('driver reset ACK invalid: '+str(error));return
        ack=pending['ack'];state=self.driver_watchdog.state
        if (ack and self.driver_watchdog.motion_ready(now) and state['session']==pending['session'] and
                state['state_seq']>ack['state_seq']):
            packet=pending['packet'];self.driver_reset_pending=None
            self.core.receive(packet,now,self.get_clock().now().nanoseconds/1e9)
            if self.core.fault is None:
                self.limiter.reset(quiet=self.core.quiet(now),explicit_reset=True)
                if self.raw_tracker.error:self.raw_tracker.reset()

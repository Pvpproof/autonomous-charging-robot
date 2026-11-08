"""Nav2 PAUSE I/O on the existing base Bridge node; never creates another node."""
import time
from charging_robot.navigation.nav2_pause import Nav2PauseGate

class Nav2PauseBridge:
    def init_nav2_pause(self):
        self.nav2_pause_gate=Nav2PauseGate()
        self.nav2_pause_clients={}
        self.nav2_pause_pending=[]

    def request_nav2_pause(self):
        from charging_robot.arm.arm_tools_guard import locked_base_ready
        now=time.monotonic();state=self.snapshot()
        with self.lock:
            spec=self.nav2_pause_gate.request(now,base_locked=locked_base_ready(state,now))
            if spec is None:raise ValueError('本次Nav2暂停不能重复或恢复，请检查流程状态')
            self.nav2_pause_pending=[spec]
        self.event('nav2_pause_requested',request_id=spec['request_id'])
        return spec['request_id']

    def cancel_nav2_pause(self,reason):
        with self.lock:
            self.nav2_pause_gate.cancel(reason);self.nav2_pause_pending=[]

    def tick_nav2_pause(self):
        from charging_robot.arm.arm_tools_guard import locked_base_ready
        state=self.snapshot();now=time.monotonic();locked=locked_base_ready(state,now)
        with self.lock:
            gate=self.nav2_pause_gate
            gate.poll(now,base_locked=locked)
            permit=gate.take_arm_permit(now,base_locked=locked)
            if gate.phase in ('failed','cancelled','authorized'):
                self.nav2_pause_pending=[]
            pending=list(self.nav2_pause_pending)
        if permit:self.event('nav2_pause_verified',**permit)
        for spec in pending:
            # All clients are created in this node's ROS executor tick.
            key=spec['service']
            if key not in self.nav2_pause_clients:
                if spec.get('action')=='pause':
                    from nav2_msgs.srv import ManageLifecycleNodes
                    typ=ManageLifecycleNodes
                else:
                    from lifecycle_msgs.srv import GetState
                    typ=GetState
                self.nav2_pause_clients[key]=(self.create_client(typ,key),typ)
            client,typ=self.nav2_pause_clients[key]
            if not client.service_is_ready():continue
            with self.lock:
                if spec not in self.nav2_pause_pending:continue
                self.nav2_pause_pending.remove(spec)
                if self.nav2_pause_gate.phase not in ('pausing','checking_states'):continue
            request=typ.Request()
            if spec.get('action')=='pause':request.command=spec['command']
            try:
                future=client.call_async(request)
                future.add_done_callback(lambda f,s=spec:self._nav2_pause_response(s,f))
            except Exception as error:self._nav2_pause_error(spec,str(error))

    def _nav2_pause_error(self,spec,reason):
        with self.lock:
            if spec.get('action')=='pause':
                self.nav2_pause_gate.ack_pause(spec['request_id'],False,time.monotonic(),reason)
            else:self.nav2_pause_gate.query_failed(spec['request_id'],spec['query_id'],spec['node'],reason)

    def _nav2_pause_response(self,spec,future):
        try:
            response=future.result()
            with self.lock:
                if spec.get('action')=='pause':
                    queries=self.nav2_pause_gate.ack_pause(spec['request_id'],response.success,time.monotonic())
                    if queries:
                        self.nav2_pause_pending=[dict(q,request_id=queries['request_id'],query_id=queries['query_id']) for q in queries['requests']]
                else:self.nav2_pause_gate.record_state(spec['request_id'],spec['query_id'],spec['node'],response.current_state.id,time.monotonic())
        except Exception as error:self._nav2_pause_error(spec,str(error))

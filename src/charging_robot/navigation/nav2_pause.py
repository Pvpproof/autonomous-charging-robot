"""Single-use Nav2 navigation PAUSE confirmation; pure data, no ROS or process I/O.

The existing panel/Bridge owns the clients and keeps checking its established
base-lock, raw-odometry and route/session guards. Pass their *current* result as
base_locked to request/poll/take_arm_permit. This helper never pauses drivers,
localization, a container or an OS process, and never resumes anything.

Protocol:
  spec = gate.request(now, base_locked=locked_base_ready(...))
  # Bridge sends ManageLifecycleNodes(command=spec['command']) on spec['service'].
  queries = gate.ack_pause(spec['request_id'], response.success, now)
  # Only now issue these GetState requests. Capture request_id/query_id/node in
  # each callback; pass response.current_state.id to record_state.
  gate.record_state(request_id, query_id, node, state_id, now)
  permit = gate.take_arm_permit(now, base_locked=current_outer_guard_result)
  # A permit authorizes the caller to launch its already-confirmed arm workflow
  # once. It does not prove physical safety or replace the caller's safeguards.
"""
import math
import uuid


NAVIGATION_NODES = (
    '/controller_server', '/smoother_server', '/planner_server',
    '/behavior_server', '/bt_navigator', '/waypoint_follower',
    '/local_costmap/local_costmap', '/global_costmap/global_costmap',
)
LOCALIZATION_NODES = ('/map_server', '/amcl')
EXPECTED_STATES = dict.fromkeys(NAVIGATION_NODES, 2)  # lifecycle INACTIVE
EXPECTED_STATES.update(dict.fromkeys(LOCALIZATION_NODES, 3))  # keep ACTIVE
PAUSE_SERVICE = '/lifecycle_manager_navigation/manage_nodes'


def _finite(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, ValueError):
        return False


class Nav2PauseGate:
    TOTAL_TIMEOUT_S = 10.
    STATE_FRESH_S = 2.
    TERMINAL = ('failed', 'cancelled', 'authorized')

    def __init__(self):
        self.request_id = uuid.uuid4().hex
        self.query_id = None
        self.phase = 'idle'
        self.reason = None
        self.started_at = None
        self.last_at = None
        self.states = {}
        self.state_times = {}

    def snapshot(self):
        return dict(request_id=self.request_id, query_id=self.query_id,
                    phase=self.phase, reason=self.reason,
                    received_states=dict(self.states), expected_states=dict(EXPECTED_STATES))

    def fail(self, reason):
        if self.phase not in self.TERMINAL:
            self.phase = 'failed'
            self.reason = str(reason)
        return self.snapshot()

    def cancel(self, reason='operator cancelled Nav2 pause workflow'):
        if self.phase not in self.TERMINAL:
            self.phase = 'cancelled'
            self.reason = str(reason)
        return self.snapshot()

    def _clock(self, now):
        if not _finite(now) or (self.last_at is not None and now < self.last_at):
            self.fail('Nav2 pause clock invalid or reversed')
            return False
        self.last_at = now
        if self.started_at is not None and now-self.started_at > self.TOTAL_TIMEOUT_S:
            self.fail('Nav2 pause/state confirmation timed out')
            return False
        return True

    def poll(self, now, *, base_locked):
        """Outer panel calls every tick with its existing live guard result."""
        if self.phase == 'idle' or self.phase in self.TERMINAL:
            return self.snapshot()
        if not self._clock(now):
            return self.snapshot()
        if base_locked is not True:
            return self.fail('base stop/lock or session guard lost during Nav2 pause')
        if self.phase == 'paused' and any(now-t > self.STATE_FRESH_S for t in self.state_times.values()):
            return self.fail('confirmed lifecycle state snapshot expired')
        return self.snapshot()

    def request(self, now, *, base_locked):
        """Emit PAUSE once, only after the caller verified a locked, stopped base."""
        if self.phase != 'idle':
            return None
        if not self._clock(now):
            return None
        if base_locked is not True:
            self.fail('verified base stop/lock required before Nav2 pause')
            return None
        self.started_at = now
        self.phase = 'pausing'
        return dict(action='pause', request_id=self.request_id,
                    service=PAUSE_SERVICE, service_type='nav2_msgs/srv/ManageLifecycleNodes',
                    command=1)  # installed ManageLifecycleNodes.PAUSE

    def ack_pause(self, request_id, success, now, reason=None):
        """Only this request's successful ACK opens a new state-query round."""
        if request_id != self.request_id or self.phase != 'pausing':
            return None
        if not self._clock(now):
            return None
        if success is not True:
            self.fail('Nav2 PAUSE rejected: '+str(reason or 'service did not confirm success'))
            return None
        self.query_id = uuid.uuid4().hex
        self.phase = 'checking_states'
        return dict(action='query_states', request_id=self.request_id, query_id=self.query_id,
                    requests=[dict(node=node, service=node+'/get_state',
                                   service_type='lifecycle_msgs/srv/GetState', expected_state=state)
                              for node, state in EXPECTED_STATES.items()])

    def record_state(self, request_id, query_id, node, state_id, now):
        """Record GetState ACKs from the post-PAUSE query round only."""
        if (request_id != self.request_id or query_id != self.query_id or
                self.phase != 'checking_states'):
            return False
        if not self._clock(now):
            return False
        if node not in EXPECTED_STATES:
            self.fail('unexpected lifecycle node in pause confirmation: '+str(node))
            return False
        if type(state_id) is not int or state_id != EXPECTED_STATES[node]:
            self.fail('unexpected lifecycle state for {}: {!r}; expected {}'.format(
                node, state_id, EXPECTED_STATES[node]))
            return False
        if node in self.states:
            return False  # duplicate callback never refreshes an old result
        self.states[node] = state_id
        self.state_times[node] = now
        if len(self.states) == len(EXPECTED_STATES):
            if max(self.state_times.values())-min(self.state_times.values()) > self.STATE_FRESH_S:
                self.fail('lifecycle query round did not form a fresh state snapshot')
                return False
            self.phase = 'paused'
        return True

    def query_failed(self, request_id, query_id, node, reason):
        if (request_id == self.request_id and query_id == self.query_id and
                self.phase == 'checking_states'):
            self.fail('lifecycle state query failed for {}: {}'.format(node, reason))
        return self.snapshot()

    def take_arm_permit(self, now, *, base_locked):
        """Consume one permit after PAUSE + all 10 states + current outer guard."""
        if self.phase != 'paused':
            return None
        self.poll(now, base_locked=base_locked)
        if self.phase != 'paused':
            return None
        self.phase = 'authorized'
        return dict(action='arm_permitted', request_id=self.request_id,
                    query_id=self.query_id, verified_states=dict(self.states))

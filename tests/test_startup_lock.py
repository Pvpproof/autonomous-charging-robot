"""Run deployed/candidate methods without ROS, sockets, or robot interfaces."""
import ast
import importlib.util
import json
from pathlib import Path
import queue
import threading
import time
import types
import unittest

ROOT=Path(__file__).resolve().parent.parent
class String:
    def __init__(self):self.data=''
class Twist:pass
class RawReturnRuntime:pass

def methods(path,klass,names,env):
    tree=ast.parse(path.read_text());c=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==klass)
    wanted=[n for n in c.body if isinstance(n,ast.FunctionDef) and n.name in names]
    ns=dict(env);exec(compile(ast.Module(body=wanted,type_ignores=[]),str(path),'exec'),ns)
    return {n.name:ns[n.name] for n in wanted}

def bridge(candidate=True):
    path=ROOT if candidate else Path(__file__).parent/"baseline"
    env=dict(json=json,time=time,String=String,Twist=Twist,RawReturnRuntime=RawReturnRuntime)
    chosen=methods(path/'return_bridge.py','ReturnBridge',{'on_arbiter','control_tick'},env)
    chosen.update(methods(path/'ros_bridge.py','Bridge',{'release_stop','emergency_stop'},env))
    B=type('B',(),chosen);b=B();b.lock=threading.RLock();b.commands=queue.Queue();b.events=[]
    b.event=lambda kind,**kw:b.events.append((kind,kw));b.manual_pub=types.SimpleNamespace(publish=lambda x:None)
    b.software_stop=True;b.stop_reset_pending=None;b.reset_arbiter=False;b.manual=False;b.manual_target=(0.,0.)
    b.manual_stop_pending=False;b.cancel_requested=False;b.return_runtime=None;b.docking_hold=False
    b.busy=False;b.server_active=False;b.odom_time=time.monotonic();b.velocity=(0.,0.)
    b.stop_observation=types.SimpleNamespace(first=1,duration=lambda first:2.5)
    b.known_arbiter_instance='A';b.known_raw_arbiter_epoch=None
    b.raw_tracker=types.SimpleNamespace(error=None,bind=lambda x:None,reset=lambda:None)
    b.return_active=lambda:False;b.control_owner='P';b.control_sequence=10;b.control_mode='stop'
    b.return_trial_id=None;b.return_short_test=False;b.return_held=False;b.gui_at=None
    b.return_state={};b._snapshot=lambda now:{};b.get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=int(time.monotonic()*1e9)))
    b.packets=[];b.authority_pub=types.SimpleNamespace(publish=lambda m:b.packets.append(json.loads(m.data)))
    b.arbiter_state=status(fault='unexpected chassis command graph');b.arbiter_at=time.monotonic()
    return b

def status(**overrides):
    d=dict(instance_id='A',mode='stop',commissioned=True,fault=None,owner='P',authority_sequence=10,
           output_v_m_s=0.,output_w_rad_s=0.,graph_ready=True,graph_reason=None)
    d.update(overrides);return d

def deliver(b,**kw):b.on_arbiter(types.SimpleNamespace(data=json.dumps(status(**kw))))

class ResetTests(unittest.TestCase):
    def test_deployed_race_reproduced(self):
        b=bridge(False);b.release_stop();self.assertFalse(b.software_stop)
        deliver(b,fault='unexpected chassis command graph');deliver(b)
        self.assertTrue(b.software_stop,'deployed code remains locked after arbiter recovery')
    def test_pending_stays_locked_then_exact_ack_unlocks(self):
        b=bridge();b.release_stop();self.assertTrue(b.software_stop)
        deliver(b,fault='old fault');self.assertTrue(b.software_stop)
        b.control_tick();packet=b.packets[-1];self.assertTrue(packet['reset']);self.assertEqual((packet['mode'],packet['v'],packet['w']),('stop',0.,0.))
        deliver(b,fault='old fault',authority_sequence=packet['sequence']-1);self.assertTrue(b.software_stop)
        deliver(b,authority_sequence=packet['sequence']);self.assertFalse(b.software_stop);self.assertIsNone(b.stop_reset_pending)
    def test_unsolicited_healthy_status_cannot_unlock(self):
        b=bridge();deliver(b);self.assertTrue(b.software_stop)
    def test_before_reset_publication_cannot_unlock(self):
        b=bridge();b.release_stop();deliver(b,authority_sequence=100);self.assertTrue(b.software_stop)
    def test_emergency_cancels_pending(self):
        b=bridge();b.release_stop();b.control_tick();seq=b.packets[-1]['sequence'];b.emergency_stop()
        deliver(b,authority_sequence=seq+1);self.assertTrue(b.software_stop);self.assertIsNone(b.stop_reset_pending)
    def test_new_instance_cannot_unlock(self):
        b=bridge();b.release_stop();b.control_tick();deliver(b,instance_id='B',authority_sequence=100)
        self.assertTrue(b.software_stop);self.assertIsNone(b.stop_reset_pending)
    def test_timeout_keeps_locked_without_retry(self):
        b=bridge();b.release_stop();b.stop_reset_pending['requested_at']-=3.;b.control_tick()
        self.assertTrue(b.software_stop);self.assertIsNone(b.stop_reset_pending);self.assertFalse(b.packets[-1]['reset'])
    def test_rejected_ack_remains_locked(self):
        for bad in [dict(fault='actual graph error'),dict(owner='other'),dict(mode='return'),dict(output_v_m_s=-.1),dict(graph_ready=False)]:
            with self.subTest(bad=bad):
                b=bridge();b.release_stop();b.control_tick();deliver(b,authority_sequence=b.packets[-1]['sequence'],**bad)
                self.assertTrue(b.software_stop);self.assertIsNone(b.stop_reset_pending)
    def test_graph_not_ready_blocks_reset(self):
        b=bridge();b.arbiter_state.update(graph_ready=False,graph_reason='driver gate not yet verified')
        with self.assertRaisesRegex(ValueError,'driver gate'):b.release_stop()
        self.assertFalse(b.reset_arbiter);self.assertTrue(b.software_stop)
    def test_duplicate_click_rejected(self):
        b=bridge();b.release_stop()
        with self.assertRaisesRegex(ValueError,'正在等待'):b.release_stop()
    def test_stale_status_blocks_reset(self):
        b=bridge();b.arbiter_at-=1
        with self.assertRaisesRegex(ValueError,'仲裁反馈'):b.release_stop()
    def test_moving_or_busy_blocks_reset(self):
        for attr,value in [('busy',True),('velocity',(.02,0.)),('odom_time',0.)]:
            b=bridge();setattr(b,attr,value)
            with self.assertRaises(ValueError):b.release_stop()

class GraphTests(unittest.TestCase):
    def setUp(self):
        env=dict(time=time,SOURCE='raw',boot_id=lambda:'boot')
        method=methods(ROOT/'control_arbiter_node.py','ArbiterNode',{'graph_ok'},env)['graph_ok']
        self.node=types.SimpleNamespace(prefix='/charging_control',isolated=False,driver_process='driver',driver_watchdog_required=False,
            driver_gate_verified=True,driver_gate_at=time.monotonic(),relay_instance=None,core=types.SimpleNamespace(mode='stop'),
            localizer_process=None,raw_tracker=types.SimpleNamespace(bind=lambda x:None))
        self.node.get_name=lambda:'charging_control_arbiter'
        self.pubs={'/charging_control/chassis':['charging_control_arbiter'],'/odom':['ychd_robot'],
                   '/charging_control/raw_odometry':['charging_raw_odom_relay'],'/charging_control/authority':['charging_navigation_panel']}
        def infos(names):return [types.SimpleNamespace(node_namespace='/',node_name=n,endpoint_gid=b'x'*24) for n in names]
        self.node.get_publishers_info_by_topic=lambda topic:infos(self.pubs.get(topic,[]))
        self.sinks=['ychd_robot'];self.node.get_subscriptions_info_by_topic=lambda topic:infos(self.sinks)
        self.node.get_subscriber_names_and_types_by_node=lambda *a:[]
        self.node.get_service_names_and_types_by_node=lambda *a:[]
        self.check=lambda:method(self.node)
    def test_healthy_graph(self):self.assertTrue(self.check());self.assertTrue(self.node.graph_ready);self.assertIsNone(self.node.graph_reason)
    def test_missing_startup_driver_reports_exact_reason(self):
        self.sinks.clear();self.assertFalse(self.check());self.assertEqual(self.node.graph_reason,'chassis subscribers: []')
    def test_driver_handshake_reports_exact_reason(self):
        self.node.driver_gate_verified=False;self.assertFalse(self.check());self.assertIn('gate parameter',self.node.graph_reason)
    def test_extra_sink_is_rejected(self):
        self.sinks.append('observer');self.assertFalse(self.check());self.assertIn('observer',self.node.graph_reason)
    def test_bypass_rejected(self):
        self.pubs['/cmd_vel']=['teleop'];self.assertFalse(self.check());self.assertIn('bypass',self.node.graph_reason)
    def test_localizer_required_only_during_navigation(self):
        self.assertTrue(self.check());self.node.core.mode='navigation';self.assertFalse(self.check());self.assertIn('AMCL',self.node.graph_reason)
    def test_core_still_latches_graph_fault(self):
        spec=importlib.util.spec_from_file_location('tested_arbiter',ROOT/'control_arbiter.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
        c=m.Arbiter();self.assertEqual(c.tick(0,graph_ok=False),(0.,0.));self.assertEqual(c.fault,'unexpected chassis command graph')
        self.assertEqual(c.tick(1,graph_ok=True),(0.,0.));self.assertIsNotNone(c.fault)

if __name__=='__main__':unittest.main(verbosity=2)

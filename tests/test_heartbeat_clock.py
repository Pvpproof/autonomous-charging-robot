"""Force the GUI to update its heartbeat while the control thread waits for lock."""
import json,math,time,types,unittest
from pathlib import Path
from test_startup_lock import bridge,methods,String,RawReturnRuntime

ROOT=Path(__file__).parent.parent

class ClockOrderingTests(unittest.TestCase):
 def test_contended_lock_reproduces_old_negative_age_and_new_code_keeps_running(self):
  for fixed in (False,True):
   with self.subTest(fixed=fixed):
    b=bridge();clock=types.SimpleNamespace(value=10.)
    class Runtime(RawReturnRuntime):
     def __init__(self):
      self.runner=self;self.phase='running';self.activation_mode='click';self.run_session='1'*32;self.run_sequence=3;self.started=9.;self.heartbeat_at=10.
      self.plan=dict(cycle_id='c',goal_id='g',poses=[[1.,0.,0.],[0.,0.,0.]],scope=dict(source_key='key',arbiter_raw_epoch='epoch'))
     def stop(self,reason):
      self.phase='stopped';return dict(phase=self.phase,reason=reason,v=0.,w=0.)
     def tick(self,state,now,owns_control):
      if not 0<=now-self.heartbeat_at<=.25:return self.stop('future heartbeat')
      return dict(phase='running',v=-.05,w=0.)
    r=Runtime();b.return_runtime=r;b.software_stop=False;b.gui_at=10.;b.control_mode='return';b.return_arbiter_instance='A';b.return_prepared_at=9.
    b.return_active=lambda:r.phase not in ('stopped','complete')
    b.arbiter_state=dict(owner='P',commissioned=True,instance_id='A',mode='return',fault=None)
    b._snapshot=lambda now:dict(raw_odom={},busy=False)
    b.get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=int(clock.value*1e9)))
    class ContendedLock:
     def __enter__(self):
      # Deterministically model the GUI winning the lock and updating first.
      clock.value=10.1;b.gui_at=10.1;b.arbiter_at=10.1;r.heartbeat_at=10.1
     def __exit__(self,*args):return False
    b.lock=ContendedLock()
    source=ROOT/'return_bridge.py' if fixed else Path(__file__).parent/'baseline/return_bridge.py'
    env=dict(time=types.SimpleNamespace(monotonic=lambda:clock.value),json=json,math=math,String=String,RawReturnRuntime=RawReturnRuntime,SOURCE='raw')
    tick=methods(source,'ReturnBridge',{'control_tick'},env)['control_tick']
    tick(b)
    if fixed:
     self.assertEqual(r.phase,'running');self.assertEqual(b.packets[-1]['mode'],'return');self.assertFalse(b.software_stop)
    else:
     self.assertEqual(r.phase,'stopped');self.assertEqual(b.packets[-1]['mode'],'stop');self.assertTrue(b.software_stop)
if __name__=='__main__':unittest.main(verbosity=2)

"""Fresh recording read, strict real data ages. No ROS or hardware."""
import ast,copy,json,math,tempfile,types,unittest
from pathlib import Path
from unittest.mock import patch
import app
from raw_odom_mode import raw_timing,sample_now,RawChassisSession
from test_raw_mode import snap,update
from _recording_helpers import session_fixture
from _current_fixtures import ui
import return_bridge as backend

ROOT=Path(__file__).resolve().parents[1]

def stream(t,now,n=10):
 for _ in range(n):now+=.05;update(t,now)
 return now

class TimingTests(unittest.TestCase):
 def test_unchanged_limits_reject_stale_future_source_and_receive_times(self):
  with tempfile.TemporaryDirectory() as tmp:
   t,now,s=session_fixture(tmp);good=snap(t,now);self.assertEqual(sample_now(good,now)['sequence'],t.sequence)
   for received,source in ((now-.251,now),(now+.001,now),(now,now-.251),(now,now+.001)):
    bad=copy.deepcopy(good);bad['raw_odom'].update(received_at=received,source_stamp=source)
    with self.assertRaisesRegex(ValueError,'过期'):sample_now(bad,now)
 def test_diagnostics_report_ages_without_retimestamping_or_mutation(self):
  with tempfile.TemporaryDirectory() as tmp:
   t,now,s=session_fixture(tmp);d=snap(t,now);d.update(snapshot_taken_at=now,snapshot_completed_at=now+.01);before=copy.deepcopy(d);e=raw_timing(d,now+.4)
   self.assertEqual(d,before);self.assertAlmostEqual(e['receive_age_s'],.4);self.assertAlmostEqual(e['snapshot_age_s'],.4);self.assertEqual(e['source_age_at_snapshot_s'],0);self.assertEqual(e['limit_s'],.25)
 def test_cached_snapshot_is_rejected_even_while_provider_has_new_data(self):
  with tempfile.TemporaryDirectory() as tmp:
   t,now,s=session_fixture(tmp);cached=snap(t,now);now=stream(t,now);s.update(cached,'meta',now)
   self.assertEqual(s.phase,'blocked');self.assertGreater(t.sequence,s.cursor)
   e=json.loads((s.directory/'first-rejection.json').read_text())['timing'];self.assertGreater(e['receive_age_s'],.25)
 def test_raw_snapshot_retains_last_timing_when_true_stream_stops(self):
  with tempfile.TemporaryDirectory() as tmp:
   t,now,s=session_fixture(tmp);b=types.SimpleNamespace(raw_tracker=t,control_owner='panel',get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=int((now+.5)*1e9))))
   result=backend.ReturnBridge.raw_snapshot(b,now+.5);self.assertIsNone(result['raw_odom']);self.assertTrue(result['raw_odom_error']);e=raw_timing(result,now+.5);self.assertAlmostEqual(e['receive_age_s'],.5);self.assertEqual(e['sequence'],t.sequence)
   with self.assertRaises(ValueError):sample_now(result,now+.5)
 def test_bridge_snapshot_adds_capture_time_without_touching_feedback_times(self):
  tree=ast.parse((ROOT/'ros_bridge.py').read_text());klass=next(n for n in tree.body if getattr(n,'name',None)=='Bridge');method=next(n for n in klass.body if getattr(n,'name',None)=='snapshot');namespace={'time':types.SimpleNamespace(monotonic=iter([10.,10.02]).__next__)}
  exec(compile(ast.Module(body=[method],type_ignores=[]),'<isolated-snapshot>','exec'),namespace)
  import threading
  obj=types.SimpleNamespace(lock=threading.Lock(),_snapshot=lambda now:{'raw_odom':{'received_at':9.99,'source_stamp':20}})
  result=namespace['snapshot'](obj);self.assertEqual(result['snapshot_taken_at'],10.);self.assertEqual(result['snapshot_completed_at'],10.02);self.assertEqual(result['raw_odom']['received_at'],9.99)

class UiFreshnessTests(unittest.TestCase):
 def setUp(self):
  ui.WindowTests.setUp(self)
  original=self.b.emergency_stop
  self.b.emergency_stop=lambda **kwargs:original()
 tearDown=ui.WindowTests.tearDown
 def fixture(self):
  self.t,self.now,self.session=session_fixture(self.tmp.name);self.w.chassis_session=self.session;self.w.navigation_map_matches=lambda:True;self.w.document=types.SimpleNamespace(fingerprint='meta',yaml_path=Path(self.tmp.name)/'map.yaml');self.b.s.update(snap(self.t,self.now));self.w.latest=self.b.snapshot()
 def execute(self):
  with patch('app.time.monotonic',return_value=self.now):self.w.update_route()
 def test_continuous_source_and_half_second_ui_delay_keep_all_samples(self):
  self.fixture();cached=copy.deepcopy(self.w.latest);first=self.session.cursor;self.now=stream(self.t,self.now,10);self.b.s.update(snap(self.t,self.now));self.w.latest=cached
  self.execute();self.assertEqual(self.session.phase,'outbound');self.assertEqual(self.session.cursor,self.t.sequence);self.assertEqual(len(self.session.samples),11);self.assertGreater(self.session.cursor,first);self.assertFalse(self.session.rejection)
  self.w.events.flush();events=[json.loads(l) for l in (self.w.logdir/'events.jsonl').read_text().splitlines()];audit=next(e for e in events if e['text']=='raw_recording_refreshed_stale_ui_snapshot');self.assertGreater(audit['cached']['receive_age_s'],.25);self.assertLessEqual(audit['current']['receive_age_s'],.25)
 def test_ui_delay_with_true_no_new_data_still_stops_and_saves_ages(self):
  self.fixture();self.now+=.5;self.execute();self.assertEqual(self.session.phase,'blocked');self.assertTrue(self.b.software_stop)
  rejection=json.loads((self.session.directory/'first-rejection.json').read_text());self.assertGreater(rejection['timing']['receive_age_s'],.25)
 def test_stale_source_stamp_and_rejected_tracker_do_not_get_hidden_by_refresh(self):
  for cause in ('source','tracker'):
   with self.subTest(cause=cause):
    # Distinct per-run recording path, but same fake Bridge/window.
    self.fixture()
    if cause=='source':self.b.s['raw_odom']['source_stamp']-=1
    else:self.b.s['raw_odom_error']='原始里程重复、倒序或失联'
    self.execute();self.assertEqual(self.session.phase,'blocked');self.assertTrue(self.session.rejection['timing'])
    self.b.software_stop=False;self.b.s.pop('raw_odom_error',None)
 def test_successful_refresh_is_not_logged_on_every_tick(self):
  self.fixture();cached=self.w.latest;self.now=stream(self.t,self.now);self.b.s.update(snap(self.t,self.now));self.w.latest=cached;self.execute()
  self.now=stream(self.t,self.now);self.b.s.update(snap(self.t,self.now));self.execute();self.w.events.flush()
  events=[json.loads(l) for l in (self.w.logdir/'events.jsonl').read_text().splitlines()];self.assertEqual(sum(e['text']=='raw_recording_refreshed_stale_ui_snapshot' for e in events),1)

if __name__=='__main__':unittest.main()

import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
from charging_robot.control.control_arbiter import Arbiter
from charging_robot.returning.return_runtime import ReturnRuntime, check_plan
from charging_robot.returning.supervised_draft import TestConditions
from charging_robot.returning.route_cycle import CycleError
from charging_robot.returning.return_map import saved_grid_digest, grid_digest


def fixture():
    points=[dict(source_stamp=1.+i*.1,pose=[i*.01,0.,0.],odom_pose=[i*.01,0.,0.],
                 map_to_odom=[0.,0.,0.],map_grid_sha='fixture-grid') for i in range(101)]
    return dict(cycle_id='fixture-cycle',goal_id='fixture-goal',map_sha256='fixture-map',
        localization_epoch='fixture-epoch',frame_id='map',reverse_only=True,executable=False,
        source='actual_map_pose_measurements',poses=[p['pose'] for p in reversed(points)],
        coordinate_samples=points,fixture_is_synthetic=True)


def snapshot(now=10.,pose=(1.,0.,0.),v=0.,w=0.,**extra):
    s=dict(pose=pose,odom_pose=pose,map_to_odom=(0.,0.,0.),pose_stamp=1000.+now,
        pose_received_at=now,pose_age=0.,odom_tf_age=0.,alignment_age=0.,odom_age=0.,scan_age=0.,
        odom_frame='odom_combined',base_frame='base_footprint',odom_frame_valid=True,
        map_sha256='fixture-map',map_grid_sha='fixture-grid',localization_epoch='fixture-epoch',
        velocity=(v,w),stationary_since=0.,stationary_samples=100,software_stop=False,
        manual=False,manual_stop_pending=False,busy=False,docking_hold=False)
    s.update(extra);return s


def conditions(plan,**extra):
    data=dict(cycle_id=plan['cycle_id'],goal_id=plan['goal_id'],operator_present=True,route_cleared=True,
        physical_stop_ready=True,arm_stowed=True,exclusive_control_confirmed=True,
        no_rear_detection_acknowledged=True,speed_limit_m_s=.05,minimum_radius_m=1.78,
        max_time_s=60.,max_distance_m=1.3)
    data.update(extra);return TestConditions(**data)


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.store=Path(self.temp.name)/'start.json'
        self.plan=fixture()
    def tearDown(self):self.temp.cleanup()
    def build(self,**changes):
        return ReturnRuntime(self.plan,conditions(self.plan,**changes),self.store)
    def start(self,r):
        r.heartbeat(10.,True);return r.tick(snapshot(),10.,owns_control=True)
    def test_complete_closed_loop_saves_only_actual_stop(self):
        r=self.build();self.start(r);p=[1.,0.,0.];v=w=0.;still=10.
        for i in range(1,650):
            now=10.+i*.05
            p=[p[0]+v*math.cos(p[2])*.05,p[1]+v*math.sin(p[2])*.05,p[2]+w*.05]
            if abs(v)>.01 or abs(w)>.02:still=now
            r.heartbeat(now,True)
            out=r.tick(snapshot(now,tuple(p),v,w,stationary_since=still),now,owns_control=True)
            self.assertLessEqual(out['v'],0.)
            self.assertLessEqual(abs(out['w']),abs(out['v'])/1.78+1e-8)
            v,w=out['v'],out['w']
            if r.runner.phase=='complete':break
        self.assertEqual(r.runner.phase,'complete',out)
        saved=json.loads(self.store.read_text());self.assertEqual(saved['pose'],p)
        before=self.store.read_bytes()
        r.tick(snapshot(now+.05,(0.,0.,0.)),now+.05,owns_control=True)
        self.assertEqual(self.store.read_bytes(),before)
    def test_old_route_rejected(self):
        self.plan.pop('coordinate_samples')
        with self.assertRaisesRegex(CycleError,'旧记录'):self.build()
    def test_actual_endpoint_discrepancy_is_not_aligned_away(self):
        r=self.build();r.heartbeat(10.,True)
        out=r.tick(snapshot(pose=(1.69,0.,0.)),10.,owns_control=True)
        self.assertEqual(out['v'],0.);self.assertEqual(r.runner.phase,'stopped')
        self.assertIn('0.690',out['reason']);self.assertFalse(self.store.exists())
    def test_map_odom_shift_rejected_even_if_map_pose_matches(self):
        r=self.build();r.heartbeat(10.,True)
        out=r.tick(snapshot(odom_pose=(.31,0.,0.),map_to_odom=(.69,0.,0.)),10.,owns_control=True)
        self.assertIn('漂移',out['reason'])
    def test_future_alignment_correction_stops_before_effective_tf_changes(self):
        r=self.build();self.start(r);r.heartbeat(10.05,True)
        out=r.tick(snapshot(10.05,latest_map_to_odom=(.69,0.,0.)),10.05,owns_control=True)
        self.assertEqual(r.runner.phase,'stopped');self.assertEqual(out['v'],0.)
        self.assertIn('最新地图校正',out['reason'])
    def test_session_map_and_frame_failures(self):
        for change in [dict(localization_epoch='new'),dict(map_sha256='new'),dict(map_grid_sha='new'),
                       dict(odom_frame='odom'),dict(odom_frame_valid=False),dict(alignment_age=.26)]:
            with self.subTest(change=change):
                r=self.build();self.start(r);r.heartbeat(10.05,True)
                out=r.tick(snapshot(10.05,**change),10.05,owns_control=True)
                self.assertEqual(r.runner.phase,'stopped',out);self.assertEqual(out['v'],0.)
    def test_motion_and_localization_faults_latch(self):
        for change in [dict(pose=(.8,0.,0.),odom_pose=(.8,0.,0.)),dict(velocity=(.04,0.)),
                       dict(pose_age=.26),dict(odom_age=.26),dict(scan_age=2.),dict(manual=True),
                       dict(software_stop=True),dict(busy=True),dict(docking_hold=True)]:
            with self.subTest(change=change):
                r=self.build();self.start(r);r.heartbeat(10.05,True)
                out=r.tick(snapshot(10.05,**change),10.05,owns_control=True)
                self.assertEqual(r.runner.phase,'stopped',out)
                r.heartbeat(10.1,True)
                self.assertEqual(r.tick(snapshot(10.1),10.1,owns_control=True)['v'],0.)
    def test_hold_loss_and_ownership_loss(self):
        for fault in ('release','heartbeat','owner','loop'):
            r=self.build();self.start(r)
            now=10.3 if fault in ('heartbeat','loop') else 10.05
            if fault=='release':r.heartbeat(now,False)
            if fault=='loop':r.heartbeat(now,True)
            out=r.tick(snapshot(now),now,owns_control=fault!='owner')
            self.assertEqual(r.runner.phase,'stopped',out);self.assertFalse(self.store.exists())
    def test_short_test_stops_without_saving(self):
        r=self.build(max_time_s=3.,max_distance_m=.15);self.start(r)
        for i in range(1,65):
            now=10.+i*.05;r.heartbeat(now,True)
            out=r.tick(snapshot(now,(1.-i*.0025,0.,0.),-.05),now,owns_control=True)
        self.assertEqual(r.runner.phase,'stopped');self.assertFalse(self.store.exists())
    def test_no_output_before_arbiter_handoff(self):
        r=self.build();r.heartbeat(10.,True)
        out=r.tick(snapshot(),10.,owns_control=False)
        self.assertEqual(out['intent'],'AWAIT_EXCLUSIVE_HANDOFF');self.assertEqual(out['v'],0.)
    def test_corrupt_geometry_and_coordinate_sequence_rejected(self):
        for variant in ('reverse','compose','time','grid'):
            p=fixture()
            if variant=='reverse':p['poses'].reverse()
            if variant=='compose':p['coordinate_samples'][3]['odom_pose'][0]+=.2
            if variant=='time':p['coordinate_samples'][3]['source_stamp']=1.
            if variant=='grid':p['coordinate_samples'][3]['map_grid_sha']='other'
            with self.subTest(variant=variant),self.assertRaises(CycleError):check_plan(p)


class ArbiterTests(unittest.TestCase):
    def build(self):
        a=Arbiter();a.nav_active=False
        for i in range(42):a.odom(0.,0.,i*.05,i*.05)
        return a
    def packet(self,a,mode='return',now=2.1,**extra):
        p=dict(owner='panel-a',sequence=a.sequence+1,mode=mode,v=-.05 if mode=='return' else 0.,w=0.,stamp=now)
        p.update(extra);a.receive(p,now,now)
    def test_return_and_release_output(self):
        a=self.build();self.packet(a);self.assertEqual(a.tick(2.1),(-.05,0.))
        self.packet(a,'stop',2.15);self.assertEqual(a.tick(2.15),(0.,0.))
    def test_heartbeat_and_odometry_timeouts_latch(self):
        for heartbeat in (True,False):
            a=self.build();self.packet(a)
            if heartbeat:self.packet(a,now=2.4)
            self.assertEqual(a.tick(2.4),(0.,0.));self.assertIsNotNone(a.fault)
            self.packet(a,now=2.45);self.assertEqual(a.tick(2.45),(0.,0.))
    def test_competing_owner_replay_and_invalid_packets(self):
        for extra in [dict(owner='other'),dict(sequence=0),dict(v=.01),dict(v=-.051),dict(w=.1),dict(stamp=0.),dict(v=float('nan'))]:
            a=self.build();self.packet(a)
            self.packet(a,now=2.15,**extra)
            self.assertIsNotNone(a.fault,extra);self.assertEqual(a.tick(2.15),(0.,0.))
    def test_navigation_is_not_replayed_on_handoff(self):
        a=self.build();a.navigation(.2,0.,2.0);self.packet(a,'navigation')
        self.assertEqual(a.tick(2.1),(0.,0.))
        a.navigation(.2,0.,2.11);self.assertEqual(a.tick(2.11),(.2,0.))
        self.packet(a,now=2.15);self.assertIsNotNone(a.fault)
    def test_nav_active_or_graph_change_stops_return(self):
        for graph in (True,False):
            a=self.build();self.packet(a)
            if graph:a.nav_active=True
            self.assertEqual(a.tick(2.1,graph_ok=graph),(0.,0.));self.assertIsNotNone(a.fault)
    def test_reset_requires_fresh_standstill_and_never_resumes(self):
        a=self.build();self.packet(a);a.fail('test')
        self.packet(a,'stop',2.1,reset=True)
        self.assertIsNone(a.fault);self.assertEqual(a.tick(2.1),(0.,0.))
        a.odom(.1,0.,2.15,2.15);a.fail('test')
        self.packet(a,'stop',2.15,reset=True);self.assertIsNotNone(a.fault)
    def test_unknown_nav_state_refuses_return(self):
        a=self.build();a.nav_active=None;self.packet(a);self.assertIsNotNone(a.fault)
    def test_duplicate_odom_is_not_a_heartbeat(self):
        a=self.build();self.packet(a);a.odom(0.,0.,a.odom_stamp,2.1)
        self.assertEqual(a.tick(2.1),(0.,0.));self.assertIsNotNone(a.fault)


if __name__=='__main__':unittest.main()

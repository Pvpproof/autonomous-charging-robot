import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace as Obj
from charging_robot.returning.chassis_route import ChassisSession
from charging_robot.returning.route_cycle import StartStore
from charging_robot.returning.return_runtime import check_plan
from charging_robot.returning.return_map import saved_grid_digest, grid_digest
from test_return import snapshot

class RecordingTests(unittest.TestCase):
    def test_new_recording_keeps_paired_coordinates_and_preserves_yaw(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);store=root/'start.json'
            StartStore(store).write('fixture-map',(0.,0.,0.),'manual_reference')
            s=snapshot(10.,(0.,0.,0.))
            session=ChassisSession('fixture-map',store,root,s,10.)
            for i in range(1,121):
                now=10.+i*.1;x=min(.3,i*.005)
                s=snapshot(now,(x,0.,0.),.05 if i<60 else 0.,navigation_id=session.goal_id)
                if i>=61:
                    s.update(goal_status=4,goal_finished_at=16.1,stationary_since=16.1)
                session.update(s,'fixture-map',now)
            self.assertEqual(session.core.phase,'outbound_stopped')
            plan=session.return_candidate();check_plan(plan)
            cycle=json.loads((session.directory/'cycle.json').read_text())
            self.assertEqual(len(cycle['trace']),len(cycle['coordinate_samples']))
            self.assertEqual(plan['poses'][0],[.3,0.,0.]);self.assertEqual(plan['poses'][-1],[0.,0.,0.])
            self.assertFalse(cycle['return_execution_enabled']);self.assertFalse(plan['executable'])
    def test_missing_pair_does_not_silently_upgrade_old_records(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);store=root/'start.json';StartStore(store).write('fixture-map',(0.,0.,0.),'manual_reference')
            s=snapshot(10.,(0.,0.,0.));s.pop('odom_pose')
            session=ChassisSession('fixture-map',store,root,s,10.)
            self.assertIsNotNone(session.coordinate_error)
            self.assertEqual(session.coordinate_samples,[])
    def test_saved_map_matches_ros_grid_including_origin_and_y_flip(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            (root/'map.pgm').write_bytes(b'P5\n3 2\n255\n'+bytes([254,0,205,0,254,205]))
            path=root/'map.yaml';path.write_text('image: map.pgm\nresolution: 0.05\norigin: [1, 2, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
            grid=Obj(header=Obj(frame_id='map'),info=Obj(width=3,height=2,resolution=.05,
                origin=Obj(position=Obj(x=1.,y=2.),orientation=Obj(x=0.,y=0.,z=0.,w=1.))),
                data=[100,0,-1,0,100,-1])
            self.assertEqual(saved_grid_digest(path),grid_digest(grid))
            grid.data[0]=0;self.assertNotEqual(saved_grid_digest(path),grid_digest(grid))

if __name__=='__main__':unittest.main(verbosity=2)

"""75 mm geometry and actual executor preflight, without ROS or motion."""
import ast,json,sys,tempfile,types,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1]/'arm_stack';sys.path.insert(0,str(ROOT/'tools'))
from insertion_limits import INSERTION_DEPTH_M,MAX_INSERTION_DEPTH_MM,checked_depth_mm
from insertion_geometry import seeded_cartesian_path,axis_report

class DepthTests(unittest.TestCase):
 def test_units(self):
  self.assertEqual(INSERTION_DEPTH_M,.075);self.assertEqual(MAX_INSERTION_DEPTH_MM,75.);self.assertEqual(checked_depth_mm('75'),75.)
 def test_outside_nonfinite_and_boolean_refused(self):
  for value in (0,-1,75.0001,80,True,float('nan'),float('inf')):
   with self.assertRaises(ValueError):checked_depth_mm(value)
 def test_actual_refinement_ends_at75_with_no_overshoot(self):
  distances=[]
  def solve(distance,seed):distances.append(distance);return np.array([distance,0,0,0,0,0])
  path=seeded_cartesian_path(np.zeros(6),INSERTION_DEPTH_M,solve)
  self.assertEqual(len(distances),75);self.assertAlmostEqual(path[-1][0],.075);self.assertLessEqual(max(distances),.075)
  report=axis_report(np.eye(4),[0,0,1],INSERTION_DEPTH_M);self.assertEqual(report['travel_mm'],75.)
 def preflight(self,depth=75.,**changes):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);d=dict(planning_ready=True,insertion_depth_mm=depth,source_gun_plan_id='gun',cycle_id='cycle');d.update(changes)
   p=root/'pending_insertion_trajectory.json';p.write_text(json.dumps(d));(root/'last_dispatched_trajectory.json').write_text(json.dumps(dict(plan_id='gun',cycle_id='cycle')))
   tree=ast.parse((ROOT/'tools/insert_execute.py').read_text());klass=next(n for n in tree.body if isinstance(n,ast.ClassDef));run=next(n for n in klass.body if getattr(n,'name',None)=='run');body=[]
   for n in run.body:
    if isinstance(n,ast.If) and 'isatty' in ast.unparse(n.test):break
    body.append(n)
   ns={'P':p,'ROOT':root,'json':json,'checked_depth_mm':checked_depth_mm}
   with patch.dict(sys.modules,{'prepare_insertion_target':types.SimpleNamespace(prepare=lambda:None)}):exec(compile(ast.Module(body=body,type_ignores=[]),'<real-executor-preflight>','exec'),ns)
 def test_executor_accepts75_before_confirmation(self):self.preflight()
 def test_executor_rejects_old80(self):
  with self.assertRaises(ValueError):self.preflight(80.)
 def test_plan_and_cycle_binding_preserved(self):
  for changes in ({'cycle_id':'old'},{'source_gun_plan_id':'other'},{'planning_ready':False}):
   with self.assertRaises(RuntimeError):self.preflight(**changes)
 def test_planner_uses_metres_and_serializes_millimetres(self):
  tree=ast.parse((ROOT/'recovered/plan_insertion_cartesian.py').read_text());plan=next(n for n in tree.body if getattr(n,'name',None)=='plan')
  assignment=next(n for n in ast.walk(plan) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='depth' for t in n.targets))
  depth=eval(compile(ast.Expression(assignment.value),'<depth>','eval'),{'INSERTION_DEPTH_M':INSERTION_DEPTH_M})
  field=next(n for n in ast.walk(plan) if isinstance(n,ast.keyword) and n.arg=='insertion_depth_mm')
  self.assertEqual(eval(compile(ast.Expression(field.value),'<metadata>','eval'),{'depth':depth}),75.)
if __name__=='__main__':unittest.main(verbosity=2)

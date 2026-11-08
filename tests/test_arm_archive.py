"""Static archive and pure helper checks; never starts an entry point."""
import ast,hashlib,json,re,subprocess,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]/'arm_stack'
class ArchiveTests(unittest.TestCase):
 def test_manifests_match(self):
  count=0
  for manifest in (ROOT/'source-manifest.json',ROOT/'recovered/source-manifest.json'):
   for row in json.loads(manifest.read_text())['sources']:
    p=manifest.parent/row['file'];self.assertEqual(hashlib.sha256(p.read_bytes()).hexdigest(),row['published_sha256']);self.assertFalse(row['current_field_verified']);count+=1
  self.assertEqual(count,42) # 40 historical sources/configs + two inactive JSON examples
 def test_all_python_parses_without_import(self):
  for p in ROOT.rglob('*.py'):ast.parse(p.read_text(),filename=str(p))
 def test_shell_syntax_only(self):
  for p in ROOT.rglob('*.sh'):
   out=subprocess.run(['bash','-n',str(p)],capture_output=True,text=True);self.assertEqual(out.returncode,0,out.stderr)
 def test_no_personal_paths_addresses_or_keys(self):
  for p in ROOT.rglob('*'):
   if p.is_file():
    text=p.read_text();self.assertNotRegex(text,r'/home/(?:nvidia|jetson)/|192\.168\.\d+\.\d+|BEGIN (?:RSA |OPENSSH )?PRIVATE KEY')
 def test_runtime_defaults_are_not_supplied(self):
  text=(ROOT/'tools/nova2_env.sh').read_text();self.assertIn('CHARGING_ARM_ADDRESS:?',text)
  self.assertFalse((ROOT/'vision/v3/pnp_near.json').exists());self.assertFalse((ROOT/'vision/v3/live_settings.json').exists())
  for extension in ('*.npz','*.ply','*.stl','*.bag','*.jsonl'):self.assertFalse(list(ROOT.rglob(extension)))
 def test_known_missing_modules_not_fabricated(self):
  self.assertFalse(list(ROOT.rglob('*.urdf')))
  self.assertFalse(list(ROOT.rglob('*.stl')))
 def test_reconstructions_are_distinct_and_hashed(self):
  rows=json.loads((ROOT/'source-manifest.json').read_text())['reconstructed_sources']
  self.assertEqual(len(rows),5)
  for row in rows:
   p=ROOT/row['file'];self.assertIn('reconstruct',row['origin'].lower());self.assertEqual(hashlib.sha256(p.read_bytes()).hexdigest(),row['published_sha256']);self.assertFalse(row['field_tested'])
if __name__=='__main__':unittest.main(verbosity=2)

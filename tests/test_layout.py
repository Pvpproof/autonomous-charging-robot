"""Entry-point compatibility without starting nodes or devices."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
from charging_robot.paths import PROJECT_ROOT, SOURCE_LAYOUT, source_path

class LayoutTests(unittest.TestCase):
    def test_complete_mapping_and_project_root(self):
        self.assertEqual(PROJECT_ROOT, Path(__file__).resolve().parents[1])
        self.assertEqual(len(SOURCE_LAYOUT), 42)
        for old, new in SOURCE_LAYOUT.items():
            self.assertEqual(source_path(old), PROJECT_ROOT/new)
            self.assertTrue(source_path(old).is_file(), old)
        self.assertEqual(json.loads((PROJECT_ROOT/'docs/source-layout.json').read_text()), SOURCE_LAYOUT)

    def test_legacy_cli_help_from_unrelated_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1', QT_QPA_PLATFORM='offscreen')
            for entry in ['app.py','control_arbiter_node.py','arm_tool_terminal.py','docking_console.py']:
                result=subprocess.run(['/usr/bin/python3',str(PROJECT_ROOT/entry),'--help'],cwd=tmp,env=env,capture_output=True,text=True,timeout=15)
                self.assertEqual(result.returncode,0,result.stderr)
                self.assertIn('usage:',result.stdout)
            result=subprocess.run(['bash',str(PROJECT_ROOT/'start.sh'),'--offline','--help'],cwd=tmp,env=env,capture_output=True,text=True,timeout=15)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('--offline',result.stdout)

    def test_launch_entry_only_constructs_description(self):
        spec=importlib.util.spec_from_file_location('legacy_navigation_entry',PROJECT_ROOT/'navigation.launch.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ, {"ROS_LOG_DIR": tmp}):
            description=module.generate_launch_description()
            self.assertTrue(description.entities)

    def test_public_manifest_matches_relocated_sources(self):
        manifest=json.loads((PROJECT_ROOT/'docs/source-manifest.json').read_text())
        for row in manifest['files']:
            self.assertEqual(hashlib.sha256((PROJECT_ROOT/row['file']).read_bytes()).hexdigest(),row['published_sha256'],row['file'])

if __name__=='__main__':unittest.main(verbosity=2)

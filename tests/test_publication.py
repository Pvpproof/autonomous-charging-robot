"""Verify publication changes preserve explicit configuration boundaries."""
from charging_robot.paths import source_path
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from launch import LaunchContext
from launch.actions import LogInfo

ROOT = Path(__file__).resolve().parents[1]


def navigation_module():
    spec = importlib.util.spec_from_file_location('publication_navigation', source_path('navigation.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PublicationConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.map_file = Path(self.tmp.name)/'synthetic.yaml'
        self.map_file.write_text('image: synthetic.pgm\n')
        self.context = LaunchContext()
        self.context.launch_configurations.update(mode='navigation', map=str(self.map_file))

    def tearDown(self):
        self.tmp.cleanup()

    def test_navigation_requires_explicit_local_parameters(self):
        with patch.dict(os.environ, {'CHARGING_NAV2_PARAMS': ''}):
            with self.assertRaisesRegex(RuntimeError, 'CHARGING_NAV2_PARAMS'):
                navigation_module().build(self.context)

    def test_documentation_placeholder_cannot_start_navigation(self):
        with patch.dict(os.environ, {'CHARGING_NAV2_PARAMS': str(ROOT/'examples/nav2.placeholder.yaml')}):
            with self.assertRaisesRegex(RuntimeError, '占位'):
                navigation_module().build(self.context)

    def test_reviewed_external_configuration_is_forwarded(self):
        config = Path(self.tmp.name)/'local-parameters.yaml'
        config.write_text('{}\n')
        module = navigation_module()
        with patch.dict(os.environ, {'CHARGING_NAV2_PARAMS': str(config)}), \
                patch.object(module, 'include', return_value=LogInfo(msg='not executed')) as include:
            module.build(self.context)
        self.assertEqual(include.call_args.kwargs['params_file'], str(config))
        self.assertEqual(include.call_args.kwargs['map'], str(self.map_file))

    def test_docking_command_can_be_set_without_embedded_account_path(self):
        spec = importlib.util.spec_from_file_location('publication_docking', source_path('docking_console.py'))
        module = importlib.util.module_from_spec(spec)
        with patch.dict(os.environ, {'CHARGING_DOCKING_COMMAND': '/synthetic/bin/docking-tool'}):
            spec.loader.exec_module(module)
        self.assertEqual(module.COMMAND, '/synthetic/bin/docking-tool')

    def test_published_configuration_remains_uncommissioned(self):
        config = json.loads((ROOT/'config/return_commissioning.json').read_text())
        self.assertIs(config['commissioned'], False)
        self.assertEqual(config['profile'], 'disabled')
        self.assertEqual(config['return_start_mode'], 'click')
        self.assertNotIn('full_trial_authorization', config)
        self.assertTrue(all(value is False for value in config['site_authorization'].values()))
        self.assertFalse((ROOT/'config/nav2_ackermann.yaml').exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)

"""Check the ROS test guard without importing ROS or creating any nodes."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


class IsolationGuardTests(unittest.TestCase):
    def guard(self, *, reference='host', current='private', interfaces=('lo',), devices=False):
        source = Path(__file__).with_name('legacy_ros_harness.py').read_text()
        function = next(node for node in ast.parse(source).body
                        if isinstance(node, ast.FunctionDef) and node.name == 'isolation')
        module = ast.Module(body=[function], type_ignores=[])
        read_text = Mock(return_value=reference)
        if reference is None:
            read_text.side_effect = FileNotFoundError('reference unavailable')
        namespace = dict(
            Path=lambda _: SimpleNamespace(read_text=read_text),
            os=SimpleNamespace(readlink=lambda _: current),
            socket=SimpleNamespace(if_nameindex=lambda: list(enumerate(interfaces))),
            glob=SimpleNamespace(glob=lambda _: ['synthetic-device'] if devices else []),
        )
        exec(compile(ast.fix_missing_locations(module), '<isolated-test-guard>', 'exec', optimize=2), namespace)
        return namespace['isolation']

    def test_missing_or_unsafe_isolation_is_rejected_even_with_optimization(self):
        for settings in [dict(reference=None), dict(reference=''), dict(current='host'),
                         dict(interfaces=('lo', 'eth0')), dict(devices=True)]:
            with self.subTest(settings=settings), self.assertRaises(RuntimeError):
                self.guard(**settings)()

    def test_guard_accepts_separate_loopback_only_environment_without_serial(self):
        self.guard()()


if __name__ == '__main__':
    unittest.main(verbosity=2)

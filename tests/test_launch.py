import unittest
from launch import LaunchContext
from launch_ros.actions import SetRemap, Node
from launch_ros.actions.load_composable_nodes import get_composable_node_load_request
from launch_ros.descriptions import ComposableNode

class RemappingTests(unittest.TestCase):
    def test_composed_navigation_receives_scoped_remapping(self):
        context=LaunchContext()
        SetRemap(src='/cmd_vel',dst='/charging_control/navigation').execute(context)
        description=ComposableNode(package='nav2_controller',plugin='nav2_controller::ControllerServer',
            name='controller_server',remappings=[('/tf','tf')])
        request=get_composable_node_load_request(description,context)
        self.assertIn('/cmd_vel:=/charging_control/navigation',request.remap_rules)
        self.assertIn('/tf:=tf',request.remap_rules)
    def test_driver_remapping_changes_topic_not_parameter_dictionary(self):
        context=LaunchContext()
        SetRemap(src='/cmd_vel',dst='/charging_control/chassis').execute(context)
        node=Node(executable='/bin/true',name='ychd_robot')
        node._perform_substitutions(context)
        self.assertIn(('/cmd_vel','/charging_control/chassis'),node.expanded_remapping_rules)

if __name__=='__main__':unittest.main(verbosity=2)

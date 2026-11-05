"""Independent ROS command arbiter. Defaults to an isolated test namespace."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import String
from control_arbiter import Arbiter

ROOT = Path(__file__).resolve().parent


class ArbiterNode(Node):
    def __init__(self, *, isolated=True, context=None):
        self.prefix = '/charging_return_isolated' if isolated else '/charging_control'
        self.isolated = isolated
        super().__init__('charging_control_arbiter', context=context,
                         start_parameter_services=False, enable_rosout=False)
        self.core = Arbiter()
        self.commissioned = isolated
        if not isolated:
            try:
                config = json.loads((ROOT/'config'/'return_commissioning.json').read_text())
                self.commissioned = config.get('commissioned') is True
            except (OSError, ValueError):
                pass
        self.pub = self.create_publisher(Twist, self.prefix+'/chassis', 1)
        self.status = self.create_publisher(String, self.prefix+'/status', 1)
        self.create_subscription(String, self.prefix+'/authority', self.authority, 1)
        self.create_subscription(Twist, self.prefix+'/navigation', self.navigation, 1)
        self.create_subscription(Odometry, self.prefix+'/odom' if isolated else '/odom',
                                 self.odom, qos_profile_sensor_data)
        self.create_subscription(GoalStatusArray, self.prefix+'/nav_status' if isolated else
                                 '/navigate_to_pose/_action/status', self.nav_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.05, self.tick)

    def authority(self, msg):
        try:
            self.core.receive(json.loads(msg.data), time.monotonic(),
                              self.get_clock().now().nanoseconds/1e9)
        except (ValueError, TypeError):
            self.core.fail('invalid authority JSON')

    def navigation(self, msg):
        self.core.navigation(msg.linear.x, msg.angular.z, time.monotonic())

    def odom(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        age = self.get_clock().now().nanoseconds/1e9-stamp
        valid = (msg.header.frame_id == 'odom_combined' and msg.child_frame_id == 'base_footprint'
                 and 0 <= age <= .25)
        self.core.odom(msg.twist.twist.linear.x, msg.twist.twist.angular.z,
                       stamp, time.monotonic(), valid)

    def nav_status(self, msg):
        self.core.nav_active = any(item.status in (1, 2, 3) for item in msg.status_list)

    def graph_ok(self):
        p = self.prefix
        def identities(items):
            return [(x.node_namespace, x.node_name) for x in items]
        if identities(self.get_publishers_info_by_topic(p+'/chassis')) != [('/', self.get_name())]:
            return False
        expected_sink = 'charging_return_test_sink' if self.isolated else 'ychd_robot'
        if identities(self.get_subscriptions_info_by_topic(p+'/chassis')) != [('/', expected_sink)]:
            return False
        if identities(self.get_publishers_info_by_topic(p+'/authority')) != [('/', 'charging_navigation_panel')]:
            return False
        sources = identities(self.get_publishers_info_by_topic(p+'/navigation'))
        if (any(x not in (('/', 'controller_server'), ('/', 'behavior_server')) for x in sources)
                or len(sources) != len(set(sources))):
            return False
        # A vendor teleop or legacy panel must not retain a bypass to the driver.
        if not self.isolated and self.get_publishers_info_by_topic('/cmd_vel'):
            return False
        return True

    def tick(self):
        now = time.monotonic()
        v, w = self.core.tick(now, graph_ok=self.graph_ok())
        if not self.commissioned:
            v, w = 0., 0.
        msg = Twist(); msg.linear.x = float(v); msg.angular.z = float(w)
        self.pub.publish(msg)
        state = String()
        state.data = json.dumps(dict(owner=self.core.owner, sequence=self.core.sequence,
            mode=self.core.mode, fault=self.core.fault, commissioned=self.commissioned,
            output_topic=self.prefix+'/chassis', detects_new_obstacles=False))
        self.status.publish(state)

    def destroy_node(self):
        if hasattr(self, 'pub'): self.pub.publish(Twist())
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--live', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = ArbiterNode(isolated=not args.live)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Return the real Nova2 arm to the fixed startup joint pose."""
import math
import re
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dobot_msgs_v4.srv import MovJ, GetErrorID, GetAngle, RobotMode

POSE_FILE = Path('/opt/charging-robot-local/nova2_tools/initial_arm_pose.yaml')
JOINTS = [f'joint{i}' for i in range(1, 7)]
TOLERANCE_DEG = 0.5
WAIT_TIMEOUT = 12.0


def read_pose():
    text = POSE_FILE.read_text()
    values = {}
    for name in JOINTS:
        m = re.search(rf'^\s*{name}:\s*([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)\s*$', text, re.M)
        if not m:
            raise RuntimeError(f'初始姿态缺少 {name}: {POSE_FILE}')
        values[name] = float(m.group(1))
    return values


class ReturnHome(Node):
    def __init__(self):
        super().__init__('charge_robot_return_to_initial_pose')
        self.latest = None
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.getangle = self.create_client(GetAngle, '/dobot_bringup_ros2/srv/GetAngle')
        self.mode_client = self.create_client(RobotMode, '/dobot_bringup_ros2/srv/RobotMode')
        self.movj = self.create_client(MovJ, '/dobot_bringup_ros2/srv/MovJ')
        self.error_id = self.create_client(GetErrorID, '/dobot_bringup_ros2/srv/GetErrorID')

    def _joint_cb(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(j in values for j in JOINTS):
            self.latest = [float(values[j]) for j in JOINTS]

    def query(self, client, request):
        # Only used for GetAngle, RobotMode and GetErrorID, never MovJ.
        if not client.wait_for_service(timeout_sec=1.0):
            print(f'[等待] 正在发现回位反馈服务 {client.srv_name}（最长15秒）...',flush=True)
            if not client.wait_for_service(timeout_sec=14.0):
                raise RuntimeError(f'15秒内未发现 {client.srv_name}；请检查bringup连接及ROS网络')
        for attempt in range(2):
            future=client.call_async(request)
            rclpy.spin_until_future_complete(self,future,timeout_sec=5.0)
            if future.done():
                result=future.result()
                if result is not None:
                    if result.res!=0:raise RuntimeError(f'{client.srv_name}返回失败 res={result.res}')
                    return result.robot_return
            future.cancel()
            if attempt==0:print(f'[重试读取] {client.srv_name} 暂无反馈，再读取一次',flush=True)
        raise RuntimeError(f'{client.srv_name}反馈超时；未取得可靠控制器状态')

    def wait_state(self):
        values=[float(v) for v in re.findall(r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?',self.query(self.getangle,GetAngle.Request()))]
        if len(values)!=6 or not all(math.isfinite(v) for v in values):
            raise RuntimeError('GetAngle返回格式错误')
        return [math.radians(v) for v in values]

    def robot_mode(self):
        values=re.findall(r'[-+]?\d+',self.query(self.mode_client,RobotMode.Request()))
        if len(values)!=1:raise RuntimeError('RobotMode返回格式错误')
        return int(values[0])

    def wait_arrival(self,target):
        deadline=time.monotonic()+90.0;consecutive=0
        while time.monotonic()<deadline:
            actual=self.wait_state()
            error=max(abs(math.degrees(a-b)) for a,b in zip(actual,target))
            mode=self.robot_mode()
            consecutive=consecutive+1 if error<=TOLERANCE_DEG and mode==5 else 0
            if consecutive>=3:
                self.get_logger().info('实际回位完成：连续3次GetAngle到位且RobotMode=5')
                return 0
            if mode not in (5,7,8):
                raise RuntimeError(f'回位期间RobotMode={mode}异常；未标记回位成功')
            rclpy.spin_once(self,timeout_sec=0.2)
        raise RuntimeError('等待实际回位超时；控制器可能仍在运动，请确认实际状态，勿直接重试')

    def call_error_check(self):
        result=self.query(self.error_id,GetErrorID.Request())
        if result.strip() not in ('{[]}', '[]'):
            raise RuntimeError(f'机械臂存在报警 {result}，未发送运动命令')

    def return_home(self):
        target_rad = read_pose()
        current = self.wait_state()
        mode = self.robot_mode()
        if mode not in (4,5):
            raise RuntimeError(f'当前RobotMode={mode}，禁止在运动/异常状态下回位')
        target = [target_rad[j] for j in JOINTS]
        errors_deg = [abs(math.degrees(a - b)) for a, b in zip(target, current)]
        self.get_logger().info('当前关节角(度): ' + ', '.join(f'{math.degrees(x):.3f}' for x in current))
        self.get_logger().info('初始关节角(度): ' + ', '.join(f'{math.degrees(x):.3f}' for x in target))
        self.get_logger().info('最大关节误差(度): %.3f' % max(errors_deg))
        if max(errors_deg) <= TOLERANCE_DEG:
            self.get_logger().info('已在初始姿态容差内，不发送 MovJ')
            return 0
        if mode!=5:
            raise RuntimeError('机械臂未使能，请先charge_robot enable')
        self.call_error_check()
        if not self.movj.wait_for_service(timeout_sec=15.0):
            raise RuntimeError('MovJ 服务不可用，未发送运动命令')
        req = MovJ.Request()
        req.mode = True
        req.a, req.b, req.c, req.d, req.e, req.f = [math.degrees(x) for x in target]
        req.param_value = ['user=0', 'tool=0', 'a=10', 'v=15', 'cp=100']
        self.get_logger().warn('发送一次 MovJ 回初始姿态（a=10, v=15）')
        future = self.movj.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        result = future.result()
        if result is None:
            raise RuntimeError('MovJ响应超时，无法确认控制器是否已接收；请检查实际状态，勿直接重复回位')
        if result.res != 0:
            raise RuntimeError(f'MovJ 返回失败: {result}')
        self.get_logger().info('MovJ 已被控制器接受，等待实际到位...')
        return self.wait_arrival(target)


def main():
    rclpy.init()
    node = ReturnHome()
    try:
        rc = node.return_home()
    except Exception as exc:
        node.get_logger().error(str(exc))
        rc = 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from rm_ros_interfaces.msg import Movej


JOINT_LIMITS_DEG = {
    6: [(-178.0, 178.0), (-130.0, 130.0), (-135.0, 135.0),
        (-178.0, 178.0), (-128.0, 128.0), (-360.0, 360.0)],
    7: [(-178.0, 178.0), (-130.0, 130.0), (-178.0, 178.0),
        (-135.0, 135.0), (-178.0, 178.0), (-128.0, 128.0),
        (-360.0, 360.0)],
}


class SmallJointMove(Node):
    def __init__(self):
        super().__init__("rm_small_joint_move")
        self.declare_parameter("arm_dof", 6)
        self.declare_parameter("joint_index", 6)
        self.declare_parameter("delta_deg", 2.0)
        self.declare_parameter("speed", 5)
        self.declare_parameter("execute", False)
        self.declare_parameter("state_timeout", 3.0)

        self._latest = None
        self._latest_monotonic = 0.0
        self._sent = False
        self._finished = False
        self._publisher = self.create_publisher(Movej, "/rm_driver/movej_cmd", 10)
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        self.create_subscription(Bool, "/rm_driver/movej_result", self._on_result, 10)
        self.create_timer(0.1, self._try_move)
        self.get_logger().info("等待当前关节状态；execute 默认为 false。")

    def _on_state(self, msg):
        self._latest = msg
        self._latest_monotonic = time.monotonic()

    def _ordered_positions(self, dof):
        positions = {}
        for name, position in zip(self._latest.name, self._latest.position):
            positions[name] = position
        expected = [f"joint{i}" for i in range(1, dof + 1)]
        if all(name in positions for name in expected):
            return [positions[name] for name in expected]
        if len(self._latest.position) == dof:
            return list(self._latest.position)
        raise ValueError(f"无法从 /joint_states 提取 {dof} 个机械臂关节")

    def _try_move(self):
        if self._sent or self._finished or self._latest is None:
            return
        dof = int(self.get_parameter("arm_dof").value)
        joint_index = int(self.get_parameter("joint_index").value)
        delta_deg = float(self.get_parameter("delta_deg").value)
        speed = int(self.get_parameter("speed").value)
        execute = bool(self.get_parameter("execute").value)
        timeout = float(self.get_parameter("state_timeout").value)

        try:
            if dof not in JOINT_LIMITS_DEG:
                raise ValueError("arm_dof 只能是 6 或 7")
            if not 1 <= joint_index <= dof:
                raise ValueError(f"joint_index 必须在 1..{dof} 范围内")
            if not 0.0 < abs(delta_deg) <= 3.0:
                raise ValueError("delta_deg 必须非零且绝对值不超过 3 度")
            if not 1 <= speed <= 10:
                raise ValueError("speed 必须在 1..10 范围内")
            if time.monotonic() - self._latest_monotonic > timeout:
                raise ValueError("/joint_states 已超时，拒绝运动")

            joints = self._ordered_positions(dof)
            target = list(joints)
            target[joint_index - 1] += math.radians(delta_deg)
            target_deg = math.degrees(target[joint_index - 1])
            lower, upper = JOINT_LIMITS_DEG[dof][joint_index - 1]
            margin = 2.0
            if not lower + margin <= target_deg <= upper - margin:
                raise ValueError(
                    f"目标 {target_deg:.2f} deg 太接近 J{joint_index} 限位 [{lower}, {upper}]"
                )
        except ValueError as error:
            self.get_logger().error(str(error))
            self._finished = True
            rclpy.shutdown()
            return

        summary = (
            f"J{joint_index}: {math.degrees(joints[joint_index - 1]):.2f} -> "
            f"{target_deg:.2f} deg, speed={speed}%"
        )
        if not execute:
            self.get_logger().warn(f"预演完成，未发送运动指令：{summary}")
            self.get_logger().warn("确认现场安全后，显式设置 -p execute:=true 才会执行。")
            self._finished = True
            rclpy.shutdown()
            return

        if self._publisher.get_subscription_count() == 0:
            self.get_logger().error("/rm_driver/movej_cmd 没有订阅者，拒绝发送")
            self._finished = True
            rclpy.shutdown()
            return

        msg = Movej()
        msg.joint = target
        msg.speed = speed
        msg.block = True
        msg.trajectory_connect = 0
        msg.dof = dof
        self._publisher.publish(msg)
        self._sent = True
        self.get_logger().warn(f"已发送低速小幅运动：{summary}")

    def _on_result(self, msg):
        if not self._sent:
            return
        if msg.data:
            self.get_logger().info("机械臂报告 MoveJ 执行成功。")
        else:
            self.get_logger().error("机械臂报告 MoveJ 执行失败，请查看驱动错误。")
        self._finished = True
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SmallJointMove()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

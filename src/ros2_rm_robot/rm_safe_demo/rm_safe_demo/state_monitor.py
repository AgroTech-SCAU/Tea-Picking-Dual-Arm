import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class StateMonitor(Node):
    def __init__(self):
        super().__init__("rm_state_monitor")
        self.declare_parameter("print_period", 0.5)
        self._latest = None
        self._received = False
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        period = max(0.1, float(self.get_parameter("print_period").value))
        self.create_timer(period, self._print_state)
        self.get_logger().info("等待 /joint_states；按 Ctrl+C 退出监视。")

    def _on_state(self, msg):
        self._latest = msg
        self._received = True

    def _print_state(self):
        if not self._received or self._latest is None:
            self.get_logger().warn("尚未收到 /joint_states", throttle_duration_sec=5.0)
            return
        pairs = []
        for index, position in enumerate(self._latest.position):
            name = self._latest.name[index] if index < len(self._latest.name) else f"joint{index + 1}"
            if name.startswith("joint"):
                pairs.append(f"{name}={math.degrees(position):.2f} deg")
        self.get_logger().info(" | ".join(pairs))


def main(args=None):
    rclpy.init(args=args)
    node = StateMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Empty


class StopMotion(Node):
    def __init__(self):
        super().__init__("rm_stop_motion")
        self._publisher = self.create_publisher(Empty, "/rm_driver/move_stop_cmd", 10)
        self.create_subscription(Bool, "/rm_driver/move_stop_result", self._on_result, 10)
        self._started = time.monotonic()
        self._published = False
        self.create_timer(0.05, self._tick)

    def _tick(self):
        elapsed = time.monotonic() - self._started
        if not self._published and (self._publisher.get_subscription_count() > 0 or elapsed > 0.5):
            for _ in range(3):
                self._publisher.publish(Empty())
            self._published = True
            self._started = time.monotonic()
            self.get_logger().warn("已发送轨迹停止命令。")
        elif self._published and elapsed > 2.0:
            self.get_logger().warn("未收到停止结果；请查看驱动，并准备使用实体急停。")
            rclpy.shutdown()

    def _on_result(self, msg):
        if msg.data:
            self.get_logger().info("机械臂报告轨迹停止成功。")
        else:
            self.get_logger().error("机械臂报告轨迹停止失败，请使用实体急停。")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = StopMotion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

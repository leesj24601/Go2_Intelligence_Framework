import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoRestamper(Node):
    def __init__(self) -> None:
        super().__init__("camera_info_restamper")
        self._sub = self.create_subscription(
            CameraInfo, "input_camera_info", self._callback, 10
        )
        self._pub = self.create_publisher(CameraInfo, "output_camera_info", 10)

    def _callback(self, msg: CameraInfo) -> None:
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CameraInfoRestamper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

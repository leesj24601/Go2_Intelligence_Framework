import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageRestamper(Node):
    def __init__(self) -> None:
        super().__init__("image_restamper")
        self._sub = self.create_subscription(
            Image, "input_image", self._callback, 10
        )
        self._pub = self.create_publisher(Image, "output_image", 10)

    def _callback(self, msg: Image) -> None:
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ImageRestamper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


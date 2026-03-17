from copy import deepcopy

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class RgbdRestamper(Node):
    def __init__(self) -> None:
        super().__init__("rgbd_restamper")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._rgb_pub = self.create_publisher(Image, "output_rgb", qos)
        self._depth_pub = self.create_publisher(Image, "output_depth", qos)
        self._camera_info_pub = self.create_publisher(CameraInfo, "output_camera_info", qos)

        self._rgb_sub = message_filters.Subscriber(
            self, Image, "input_rgb", qos_profile=qos
        )
        self._depth_sub = message_filters.Subscriber(
            self, Image, "input_depth", qos_profile=qos
        )
        self._camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, "input_camera_info", qos_profile=qos
        )

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._rgb_sub, self._depth_sub, self._camera_info_sub],
            queue_size=30,
            slop=0.5,
        )
        self._sync.registerCallback(self._callback)

    def _callback(self, rgb: Image, depth: Image, camera_info: CameraInfo) -> None:
        stamp = self.get_clock().now().to_msg()

        rgb_out = deepcopy(rgb)
        depth_out = deepcopy(depth)
        camera_info_out = deepcopy(camera_info)

        rgb_out.header.stamp = stamp
        depth_out.header.stamp = stamp
        camera_info_out.header.stamp = stamp

        self._rgb_pub.publish(rgb_out)
        self._depth_pub.publish(depth_out)
        self._camera_info_pub.publish(camera_info_out)


def main() -> None:
    rclpy.init()
    node = RgbdRestamper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

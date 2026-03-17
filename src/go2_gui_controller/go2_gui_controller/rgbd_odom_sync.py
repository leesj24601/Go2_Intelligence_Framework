import math
from copy import deepcopy

import message_filters
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


class RgbdOdomSync(Node):
    def __init__(self) -> None:
        super().__init__("rgbd_odom_sync")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._publish_tf = bool(self.declare_parameter("publish_tf", True).value)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        self._rgb_pub = self.create_publisher(Image, "output_rgb", qos)
        self._depth_pub = self.create_publisher(Image, "output_depth", qos)
        self._camera_info_pub = self.create_publisher(CameraInfo, "output_camera_info", qos)
        self._odom_pub = self.create_publisher(Odometry, "output_odom", qos)

        self._rgb_sub = message_filters.Subscriber(
            self, Image, "input_rgb", qos_profile=qos
        )
        self._depth_sub = message_filters.Subscriber(
            self, Image, "input_depth", qos_profile=qos
        )
        self._camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, "input_camera_info", qos_profile=qos
        )
        self._odom_sub = self.create_subscription(
            Odometry, "input_odom", self._odom_callback, qos
        )

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._rgb_sub, self._depth_sub, self._camera_info_sub],
            queue_size=30,
            slop=0.5,
        )
        self._sync.registerCallback(self._rgbd_callback)

        self._stamp_offset_ns = None
        self._latest_odom = None

    def _odom_callback(self, msg: Odometry) -> None:
        self._latest_odom = self._planarize_and_shift(msg)

    def _rgbd_callback(self, rgb: Image, depth: Image, camera_info: CameraInfo) -> None:
        if self._latest_odom is None:
            return

        stamp = self.get_clock().now().to_msg()

        rgb_out = deepcopy(rgb)
        depth_out = deepcopy(depth)
        camera_info_out = deepcopy(camera_info)
        odom_out = deepcopy(self._latest_odom)

        rgb_out.header.stamp = stamp
        depth_out.header.stamp = stamp
        camera_info_out.header.stamp = stamp
        odom_out.header.stamp = stamp

        self._rgb_pub.publish(rgb_out)
        self._depth_pub.publish(depth_out)
        self._camera_info_pub.publish(camera_info_out)
        self._odom_pub.publish(odom_out)

        if not self._tf_broadcaster:
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = odom_out.header.frame_id
        tf_msg.child_frame_id = odom_out.child_frame_id
        tf_msg.transform.translation.x = odom_out.pose.pose.position.x
        tf_msg.transform.translation.y = odom_out.pose.pose.position.y
        tf_msg.transform.translation.z = odom_out.pose.pose.position.z
        tf_msg.transform.rotation = odom_out.pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf_msg)

    def _planarize_and_shift(self, msg: Odometry) -> Odometry:
        out = deepcopy(msg)
        out.header.stamp = self._shifted_stamp(msg.header.stamp)
        out.pose.pose.position.z = 0.0

        yaw = _yaw_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        qx, qy, qz, qw = _quaternion_from_yaw(yaw)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.twist.twist.linear.z = 0.0
        out.twist.twist.angular.x = 0.0
        out.twist.twist.angular.y = 0.0
        return out

    def _shifted_stamp(self, src_stamp: TimeMsg) -> TimeMsg:
        src_ns = Time.from_msg(src_stamp).nanoseconds
        now_ns = self.get_clock().now().nanoseconds
        if self._stamp_offset_ns is None:
            self._stamp_offset_ns = now_ns - src_ns
        shifted_ns = src_ns + self._stamp_offset_ns
        return Time(nanoseconds=shifted_ns).to_msg()


def main() -> None:
    rclpy.init()
    node = RgbdOdomSync()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)

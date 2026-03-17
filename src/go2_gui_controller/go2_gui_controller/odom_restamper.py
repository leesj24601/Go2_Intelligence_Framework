import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time as TimeMsg
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import TransformBroadcaster


class OdomRestamper(Node):
    def __init__(self) -> None:
        super().__init__("odom_restamper")

        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_rate_hz", 10.0)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._publisher = self.create_publisher(Odometry, "output_odom", qos)
        self._subscription = self.create_subscription(
            Odometry,
            "input_odom",
            self._odom_callback,
            qos,
        )
        self._stamp_offset_ns = None
        self._latest_output = None
        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._publish_latest)

    def _odom_callback(self, msg: Odometry) -> None:
        stamp = self._shifted_stamp(msg.header.stamp)

        out = Odometry()
        out.header = msg.header
        out.header.stamp = stamp
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        # Indoor RTAB-Map mapping is more stable if the robot odometry is constrained
        # to planar motion. Keep x/y/yaw and remove walking-induced z/roll/pitch noise.
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
        self._latest_output = out

    def _publish_latest(self) -> None:
        if self._latest_output is None:
            return

        out = Odometry()
        out.header = self._latest_output.header
        out.header.stamp = self.get_clock().now().to_msg()
        out.child_frame_id = self._latest_output.child_frame_id
        out.pose = self._latest_output.pose
        out.twist = self._latest_output.twist
        self._publisher.publish(out)

        if not self._tf_broadcaster:
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = out.header.stamp
        tf_msg.header.frame_id = out.header.frame_id
        tf_msg.child_frame_id = out.child_frame_id
        tf_msg.transform.translation.x = out.pose.pose.position.x
        tf_msg.transform.translation.y = out.pose.pose.position.y
        tf_msg.transform.translation.z = out.pose.pose.position.z
        tf_msg.transform.rotation = out.pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf_msg)

    def _shifted_stamp(self, src_stamp: TimeMsg) -> TimeMsg:
        src_ns = Time.from_msg(src_stamp).nanoseconds
        now_ns = self.get_clock().now().nanoseconds
        if self._stamp_offset_ns is None:
            self._stamp_offset_ns = now_ns - src_ns
        shifted_ns = src_ns + self._stamp_offset_ns
        return Time(nanoseconds=shifted_ns).to_msg()


def main() -> None:
    rclpy.init()
    node = OdomRestamper()
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

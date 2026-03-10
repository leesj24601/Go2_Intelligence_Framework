from geometry_msgs.msg import Twist


class ManualControlBridge:
    def __init__(self, node, topic_name: str = "/cmd_vel"):
        self._node = node
        self._publisher = node.create_publisher(Twist, topic_name, 10)

    def send_velocity(self, linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self._publisher.publish(msg)

    def stop(self) -> None:
        for _ in range(3):
            self.send_velocity()

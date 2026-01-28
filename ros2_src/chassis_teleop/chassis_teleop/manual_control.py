#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class manual_control(Node):
    """
    Subscribes:  /joy   (sensor_msgs/Joy)
    Publishes:   /cmd_vel (geometry_msgs/Twist)

    Default Xbox mapping used by most Linux joy setups:
      A=0, B=1, X=2, Y=3

    Mapping:
      Y -> forward
      A -> backward
      X -> left (positive angular.z)
      B -> right (negative angular.z)
    """

    def __init__(self):
        super().__init__("manual_control")

        # Topics
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("cmd_vel_topic", "/diff_drive_controller/cmd_vel_unstamped")

        # Button indices (change if your controller reports differently)
        self.declare_parameter("btn_a", 0)
        self.declare_parameter("btn_b", 1)
        self.declare_parameter("btn_x", 2)
        self.declare_parameter("btn_y", 3)

        # Speeds
        self.declare_parameter("linear_speed", 0.4)   # m/s
        self.declare_parameter("angular_speed", 1.2)  # rad/s

        # Publish behavior
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("zero_when_idle", True)   # publish 0 when no buttons pressed
        self.declare_parameter("joy_timeout_sec", 0.5)    # if no joy msg recently -> stop

        self.joy_topic = self.get_parameter("joy_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.btn_a = int(self.get_parameter("btn_a").value)
        self.btn_b = int(self.get_parameter("btn_b").value)
        self.btn_x = int(self.get_parameter("btn_x").value)
        self.btn_y = int(self.get_parameter("btn_y").value)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)

        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.zero_when_idle = bool(self.get_parameter("zero_when_idle").value)
        self.joy_timeout_sec = float(self.get_parameter("joy_timeout_sec").value)

        self.sub = self.create_subscription(Joy, self.joy_topic, self.joy_cb, 10)
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.last_joy_time = None
        self.latest_buttons = []

        period = 1.0 / max(self.publish_hz, 1e-3)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"Listening on {self.joy_topic}, publishing Twist on {self.cmd_vel_topic} "
            f"(Y=forward, A=back, X=left, B=right)"
        )

    def joy_cb(self, msg: Joy):
        self.latest_buttons = list(msg.buttons)
        self.last_joy_time = self.get_clock().now()

    def _btn(self, idx: int) -> bool:
        if idx < 0:
            return False
        if idx >= len(self.latest_buttons):
            return False
        return self.latest_buttons[idx] == 1

    def on_timer(self):
        # If we haven't received a Joy message recently, stop.
        now = self.get_clock().now()
        if self.last_joy_time is None:
            if self.zero_when_idle:
                self.pub.publish(Twist())
            return

        age = (now - self.last_joy_time).nanoseconds * 1e-9
        if age > self.joy_timeout_sec:
            self.pub.publish(Twist())
            return

        forward = self._btn(self.btn_y)
        backward = self._btn(self.btn_a)
        left = self._btn(self.btn_x)
        right = self._btn(self.btn_b)

        twist = Twist()

        # Linear: Y forward, A backward (if both, cancel to 0)
        if forward and not backward:
            twist.linear.x = self.linear_speed
        elif backward and not forward:
            twist.linear.x = -self.linear_speed
        else:
            twist.linear.x = 0.0

        # Angular: X left (+), B right (-) (if both, cancel)
        if left and not right:
            twist.angular.z = self.angular_speed
        elif right and not left:
            twist.angular.z = -self.angular_speed
        else:
            twist.angular.z = 0.0

        if (twist.linear.x == 0.0 and twist.angular.z == 0.0) and self.zero_when_idle:
            # Still publish zero to keep controllers happy / actively stop
            self.pub.publish(Twist())
        else:
            self.pub.publish(twist)


def main():
    rclpy.init()
    node = manual_control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # stop on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

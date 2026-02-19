#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class manual_control(Node):
    """
    Subscribes:  /joy   (sensor_msgs/Joy)
    Publishes:   /cmd_vel (geometry_msgs/Twist)
    """

    def __init__(self):
        super().__init__("manual_control")

        # Topics
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("cmd_vel_topic", "/diff_drive_controller/cmd_vel_unstamped")

        # Speeds
        self.declare_parameter("max_linear_speed", 0.4)   # m/s
        self.declare_parameter("max_angular_speed", 1.2)  # rad/s

        # Publish behavior
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("zero_when_idle", True)   # publish 0 when no buttons pressed
        self.declare_parameter("joy_timeout_sec", 0.5)    # if no joy msg recently -> stop

        self.joy_topic = self.get_parameter("joy_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)

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
        self.axes = list(msg.axes)
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

        twist = Twist()

        AXIS_LX = 0          # left stick left/right
        AXIS_LY = 1          # left stick up/down (often inverted)
        DEADZONE = 0.10
        MAX_LIN = self.max_linear_speed    # m/s
        MAX_ANG = self.max_angular_speed   # rad/s


        lx = self.axes[AXIS_LX]
        ly = self.axes[AXIS_LY]

        forward = ly
        turn = lx

        #forward = apply_deadzone(forward, DEADZONE)
        #turn = apply_deadzone(turn, DEADZONE)

        twist = Twist()
        twist.linear.x = MAX_LIN * forward
        twist.angular.z = MAX_ANG * turn
 
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

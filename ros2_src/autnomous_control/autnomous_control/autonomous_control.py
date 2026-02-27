#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class State(Enum):
    FORWARD = 1
    AVOID_TURN = 2
    AVOID_FORWARD = 3
    RETURN_TURN = 4
    RESUME_CHECK = 5


class SimpleAutonomy(Node):
    def __init__(self):
        super().__init__("simple_autonomy")

        self.declare_parameter("cmd_vel_topic", "/diff_drive_controller/cmd_vel_unstamped")
        self.declare_parameter("object_detected_topic", "/object_detected")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("forward_speed", 0.4)
        self.declare_parameter("turn_speed", 0.8)
        self.declare_parameter("avoid_forward_speed", 0.3)
        self.declare_parameter("avoid_forward_duration", 5.0)
        self.declare_parameter("turn_duration", 3.0)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.object_detected_topic = self.get_parameter("object_detected_topic").value
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.avoid_forward_speed = float(self.get_parameter("avoid_forward_speed").value)
        self.avoid_forward_duration = float(self.get_parameter("avoid_forward_duration").value)
        self.turn_duration = float(self.get_parameter("turn_duration").value)

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.sub = self.create_subscription(
            Bool, self.object_detected_topic, self.on_object_detected, 10
        )

        self.state = State.FORWARD
        self.state_start = self.get_clock().now()
        self.object_detected = False

        self.timer = self.create_timer(1.0 / max(self.publish_hz, 1e-3), self.on_timer)
        self.get_logger().info(
            f"Autonomy publishing Twist on {self.cmd_vel_topic} and listening to {self.object_detected_topic}"
        )

    def on_object_detected(self, msg: Bool):
        self.object_detected = bool(msg.data)

    def _near_object(self) -> bool:
        return self.object_detected

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.state_start).nanoseconds * 1e-9

    def _set_state(self, new_state: State):
        self.state = new_state
        self.state_start = self.get_clock().now()

    def _publish_twist(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)

    def on_timer(self):
        match self.state:
            case State.FORWARD:
                if self._near_object():
                    self._set_state(State.AVOID_TURN)
                    self._publish_twist(0.0, 0.0)
                    return
                self._publish_twist(-self.forward_speed, 0.0)
                return

            case State.AVOID_TURN:
                if self._elapsed() >= self.turn_duration:
                    self._set_state(State.AVOID_FORWARD)
                    self._publish_twist(0.0, 0.0)
                    return
                self._publish_twist(0.0, self.turn_speed)
                return

            case State.AVOID_FORWARD:
                if self._elapsed() >= self.avoid_forward_duration:
                    self._set_state(State.RETURN_TURN)
                    self._publish_twist(0.0, 0.0)
                    return
                self._publish_twist(-self.avoid_forward_speed, 0.0)
                return

            case State.RETURN_TURN:
                if self._elapsed() >= self.turn_duration:
                    self._set_state(State.RESUME_CHECK)
                    self._publish_twist(0.0, 0.0)
                    return
                self._publish_twist(0.0, -self.turn_speed)
                return

            case State.RESUME_CHECK:
                if self._near_object():
                    self._set_state(State.AVOID_TURN)
                else:
                    self._set_state(State.FORWARD)
                self._publish_twist(0.0, 0.0)


def main():
    rclpy.init()
    node = SimpleAutonomy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist(0,0,0,0,0,0))  # Stop the robot before shutting down
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
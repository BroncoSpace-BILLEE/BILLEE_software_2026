#!/usr/bin/env python3
"""
joy_to_servo_twist.py

Subscribes:  /joy   (sensor_msgs/msg/Joy)
Publishes:   /servo_node/delta_twist_cmds (geometry_msgs/msg/TwistStamped)

Maps gamepad axes to end-effector Cartesian velocity commands:
- linear:  x, y, z  (m/s)
- angular: roll, pitch, yaw (rad/s)

Intended to drive MoveIt Servo in ROS 2 Humble.
"""

from __future__ import annotations

import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray

def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _deadband(x: float, db: float) -> float:
    return 0.0 if abs(x) < db else x


class JoyToServoTwist(Node):
    def __init__(self) -> None:
        super().__init__("joy_to_servo_twist")

        # ---- Parameters ----
        # Topics
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("servo_pub_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("wrist_pub_topic", "/wrist_controller/commands") #topic for the wrist joint controller

        # Frame
        self.declare_parameter("cmd_frame", "link_1_2_1")  # frame for the TwistStamped commands (should be a robot link)

        # Publish rate + timeout
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("joy_timeout_sec", 0.25)

        # this sets the max rates allowed by servo, not actual arm limits (set in controllers.yaml)
        self.declare_parameter("max_linear", 1.0)
        self.declare_parameter("max_angular", 1.0)
        #controller mappings
        #axes
        self.thumb_left_x_axis = 0
        self.thumb_left_y_axis = 1
        self.thumb_right_x_axis = 3
        self.thumb_right_y_axis = 4
        self.left_trigger_axis = 2
        self.right_trigger_axis = 5
        self.directional_x_axis = 6
        self.directional_y_axis = 7
        #buttons
        self.button_a = 0
        self.button_b = 1
        self.button_x = 2
        self.button_y = 3
        self.button_lb = 4
        self.button_rb = 5
        self.button_share = 6
        self.button_options = 7
        self.button_xbox = 8
        self.button_left_stick = 9
        self.button_right_stick = 10

        # ---- State ----
        self._latest_axes: List[float] = []
        self._latest_buttons: List[int] = []
        self._last_joy_time = self.get_clock().now()

        # ---- ROS I/O ----
        joy_topic = self.get_parameter("joy_topic").get_parameter_value().string_value
        # topic for the servo controller
        servo_pub_topic = self.get_parameter("servo_pub_topic").get_parameter_value().string_value

        #topic for the joint controller 
        wrist_pub_topic = self.get_parameter("wrist_pub_topic").get_parameter_value().string_value



        self._sub = self.create_subscription(Joy, joy_topic, self._on_joy, 10)
        self._servo_pub = self.create_publisher(TwistStamped, servo_pub_topic, 10)
        
        self._wrist_pub = self.create_publisher(Float64MultiArray, wrist_pub_topic, 10)

        hz = float(self.get_parameter("publish_hz").value)
        self._timer = self.create_timer(1.0 / max(hz, 1.0), self._on_timer)

        self.get_logger().info(
            f"JoyToServoTwist: joy_topic={joy_topic}, servo_pub_topic={servo_pub_topic}, cmd_frame={self.get_parameter('cmd_frame').value}"
        )

    def _on_joy(self, msg: Joy) -> None:
        self._latest_axes = list(msg.axes)
        self._latest_buttons = list(msg.buttons)
        self._last_joy_time = self.get_clock().now()

    def _btn(self, idx: int) -> bool:
        if idx < 0:
            return False
        if idx >= len(self._latest_buttons):
            return False
        return self._latest_buttons[idx] == 1

    def _axis(self, idx: int) -> float:
        if idx < 0:
            return 0.0
        if idx >= len(self._latest_axes):
            return 0.0
        return float(self._latest_axes[idx])

    def _remap(self, value, old_min, old_max, new_min, new_max):
        """
        Remaps a value from an old range to a new range.
        """
        # Ensure all values are floats for division
        old_range = float(old_max - old_min)
        new_range = float(new_max - new_min)
        
        # Calculate the normalized position (0 to 1) in the old range
        normalized_value = (value - old_min) / old_range
        
        # Scale to the new range and add the new minimum
        remapped_value = normalized_value * new_range + new_min
        
        return remapped_value

    def _on_timer(self) -> None:
        # If we haven't received joy recently, publish zeros (or stop publishing; Servo typically likes zeros).
        timeout = float(self.get_parameter("joy_timeout_sec").value)
        if (self.get_clock().now() - self._last_joy_time) > Duration(seconds=timeout):
            self._servo_publish_zero()
            return

        max_lin = float(self.get_parameter("max_linear").value)
        max_ang = float(self.get_parameter("max_angular").value)

        #translations
        #xy plane translations with left thumbstick
        vx = self._remap(self._axis(self.thumb_left_y_axis), -1, 1, max_lin, -max_lin)  # forward/back
        #vy = self._remap(self._axis(self.thumb_left_x_axis), -1, 1, -max_lin, max_lin)  # left/right
        vy = 0.0 #no strafing as the arm doesn't have that DOF

        #z translation with triggers (LT down, RT up)
        lt = self._axis(self.left_trigger_axis)  #  -1 (released) to 1 (fully pressed)
        rt = self._axis(self.right_trigger_axis)

        if lt != 1:  # if LT is pressed
            vz = self._remap(lt, 1, -1, 0, -max_lin)  # map to 0 (released) to -max_lin (fully pressed)
        elif rt != 1:  # if RT is pressed
            vz = self._remap(rt, 1, -1, 0, max_lin)   # map to 0 (released) to max_lin (fully pressed)
        elif lt == 0.0 and rt == 0.0:  # if both triggers aren't intialized
            vz = 0.0
        else:
            vz = 0.0

        #rotations
        wz = self._remap(self._axis(self.thumb_left_x_axis), -1, 1, -max_ang, max_ang)
        
        wx = self._remap(self._axis(self.thumb_right_x_axis), -1, 1, -max_ang, max_ang) 

        #rotate wrist up down with right thumbstick y axis
        wy = self._remap(self._axis(self.thumb_right_y_axis), -1, 1, -max_ang, max_ang)
        


        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("cmd_frame").value)

        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.x = float(wx)
        msg.twist.angular.y = float(wy)
        msg.twist.angular.z = float(wz)

        self._servo_pub.publish(msg)

        msg_wrist = Float64MultiArray()
        msg_wrist.data = [wx]  # Assuming the wrist controller expects a single float

        self._wrist_pub.publish(msg_wrist)


    def _servo_publish_zero(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("cmd_frame").value)
        # all zeros
        self._servo_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = JoyToServoTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
joy_to_servo_twist.py

Subscribes:  /joy   (sensor_msgs/msg/Joy)
Publishes:   /servo_node/delta_twist_cmds (geometry_msgs/msg/TwistStamped)

Maps gamepad axes to end-effector Cartesian velocity commands:
- linear:  x, y, z  (m/s)
- angular: roll, pitch, yaw (rad/s)

Intended to drive MoveIt Servo in ROS 2 Humble.

Typical MoveIt Servo expects:
  - geometry_msgs/TwistStamped on /servo_node/delta_twist_cmds
  - header.frame_id set to a command frame (often base_link)

Usage:
  ros2 run <your_pkg> joy_to_servo_twist --ros-args \
    -p cmd_frame:=base_link \
    -p publish_topic:=/servo_node/delta_twist_cmds

You must run:
  - joy_node (or equivalent)
  - move_group + moveit_servo
  - ros2_control controllers active (arm_controller)
"""

from __future__ import annotations

import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped


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
        self.declare_parameter("publish_topic", "/servo_node/delta_twist_cmds")

        # Frame
        self.declare_parameter("cmd_frame", "base_link")

        # Publish rate + timeout
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("joy_timeout_sec", 0.25)

        # Axis mapping (indices in Joy.axes)
        # Defaults are Xbox-ish; adjust to your controller.
        self.declare_parameter("axis_x", 0)         # left stick left/right
        self.declare_parameter("axis_y", 1)         # left stick up/down
        self.declare_parameter("axis_z", 5)         # right trigger (or use 2); see below
        self.declare_parameter("axis_roll", 3)      # right stick left/right
        self.declare_parameter("axis_pitch", 4)     # right stick up/down
        self.declare_parameter("axis_yaw", 2)       # left trigger (or use a stick)

        # If you want Z from two triggers, enable this:
        self.declare_parameter("use_dual_triggers_for_z", True)
        self.declare_parameter("axis_z_up_trigger", 5)    # RT
        self.declare_parameter("axis_z_down_trigger", 2)  # LT
        self.declare_parameter("triggers_are_0_to_1", False)
        # Some controllers report triggers in [-1..1] with -1 unpressed; set accordingly.

        # Scales (max speeds)
        self.declare_parameter("linear_scale", 0.15)   # m/s at full deflection
        self.declare_parameter("angular_scale", 0.6)   # rad/s at full deflection

        # Deadband + clamp
        self.declare_parameter("deadband", 0.08)
        self.declare_parameter("max_linear", 0.25)     # m/s clamp
        self.declare_parameter("max_angular", 1.2)     # rad/s clamp

        # Enable / hold-to-command
        # If enable_button < 0, always enabled.
        self.declare_parameter("enable_button", -1)    # e.g. RB=5 on Xbox
        self.declare_parameter("require_enable", False)

        # Optional: invert axes
        self.declare_parameter("invert_y", True)       # many sticks report up as +1 or -1; invert if needed
        self.declare_parameter("invert_pitch", True)

        # ---- State ----
        self._latest_axes: List[float] = []
        self._latest_buttons: List[int] = []
        self._last_joy_time = self.get_clock().now()

        # ---- ROS I/O ----
        joy_topic = self.get_parameter("joy_topic").get_parameter_value().string_value
        pub_topic = self.get_parameter("publish_topic").get_parameter_value().string_value

        self._sub = self.create_subscription(Joy, joy_topic, self._on_joy, 10)
        self._pub = self.create_publisher(TwistStamped, pub_topic, 10)

        hz = float(self.get_parameter("publish_hz").value)
        self._timer = self.create_timer(1.0 / max(hz, 1.0), self._on_timer)

        self.get_logger().info(
            f"JoyToServoTwist: joy_topic={joy_topic}, publish_topic={pub_topic}, cmd_frame={self.get_parameter('cmd_frame').value}"
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

    def _compute_z_from_triggers(self) -> float:
        # Returns z command in [-1..1] where + is up
        up_idx = int(self.get_parameter("axis_z_up_trigger").value)
        dn_idx = int(self.get_parameter("axis_z_down_trigger").value)
        t_up = self._axis(up_idx)
        t_dn = self._axis(dn_idx)

        triggers_0_to_1 = bool(self.get_parameter("triggers_are_0_to_1").value)
        if triggers_0_to_1:
            # already [0..1]
            up = _clamp(t_up, 0.0, 1.0)
            dn = _clamp(t_dn, 0.0, 1.0)
        else:
            # common: [-1..1] with -1 unpressed, +1 fully pressed
            up = (t_up + 1.0) * 0.5
            dn = (t_dn + 1.0) * 0.5
            up = _clamp(up, 0.0, 1.0)
            dn = _clamp(dn, 0.0, 1.0)

        return up - dn  # + up, - down

    def _on_timer(self) -> None:
        # If we haven't received joy recently, publish zeros (or stop publishing; Servo typically likes zeros).
        timeout = float(self.get_parameter("joy_timeout_sec").value)
        if (self.get_clock().now() - self._last_joy_time) > Duration(seconds=timeout):
            self._publish_zero()
            return

        require_enable = bool(self.get_parameter("require_enable").value)
        enable_button = int(self.get_parameter("enable_button").value)
        enabled = (not require_enable) or self._btn(enable_button)

        if not enabled:
            self._publish_zero()
            return

        db = float(self.get_parameter("deadband").value)
        lin_scale = float(self.get_parameter("linear_scale").value)
        ang_scale = float(self.get_parameter("angular_scale").value)
        max_lin = float(self.get_parameter("max_linear").value)
        max_ang = float(self.get_parameter("max_angular").value)

        invert_y = bool(self.get_parameter("invert_y").value)
        invert_pitch = bool(self.get_parameter("invert_pitch").value)

        # Linear
        ax_x = int(self.get_parameter("axis_x").value)
        ax_y = int(self.get_parameter("axis_y").value)
        ax_z = int(self.get_parameter("axis_z").value)

        x = _deadband(self._axis(ax_x), db)
        y = _deadband(self._axis(ax_y), db)
        if invert_y:
            y = -y

        use_dual = bool(self.get_parameter("use_dual_triggers_for_z").value)
        if use_dual:
            z = _deadband(self._compute_z_from_triggers(), db)
        else:
            z = _deadband(self._axis(ax_z), db)

        # Angular
        ax_r = int(self.get_parameter("axis_roll").value)
        ax_p = int(self.get_parameter("axis_pitch").value)
        ax_yaw = int(self.get_parameter("axis_yaw").value)

        roll = _deadband(self._axis(ax_r), db)
        pitch = _deadband(self._axis(ax_p), db)
        yaw = _deadband(self._axis(ax_yaw), db)

        if invert_pitch:
            pitch = -pitch

        # Scale to velocities
        vx = _clamp(x * lin_scale, -max_lin, max_lin)
        vy = _clamp(y * lin_scale, -max_lin, max_lin)
        vz = _clamp(z * lin_scale, -max_lin, max_lin)

        wx = _clamp(roll * ang_scale, -max_ang, max_ang)
        wy = _clamp(pitch * ang_scale, -max_ang, max_ang)
        wz = _clamp(yaw * ang_scale, -max_ang, max_ang)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("cmd_frame").value)

        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.x = float(wx)
        msg.twist.angular.y = float(wy)
        msg.twist.angular.z = float(wz)

        self._pub.publish(msg)

    def _publish_zero(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("cmd_frame").value)
        # all zeros
        self._pub.publish(msg)


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

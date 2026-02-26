#!/usr/bin/env python3
"""
gripper_base_teleop.py

Teleop node for a gripper-base mechanism driven by a single RoboClaw
with two channels.

Channel 1 (M1) — simple joint:
    Controlled by the left joystick X axis (axis 0).
    Full analog proportional control [-1, 1].

Channel 2 (M2) — button-driven motor:
    A button (button 0) → motor backward  (-speed)
    B button (button 1) → motor forward   (+speed)
    Neither / both      → stop (0)

The node reads /joy and publishes a synthetic sensor_msgs/Joy message
on gripper_base_joy.  The simple_roboclaw node is launched with its joy
topic remapped to gripper_base_joy.

Output Joy.axes layout:
    axes[0] = M1 speed (joint, from stick)
    axes[1] = M2 speed (gripper, from buttons)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


def _clamp(value: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, value))


class GripperBaseTeleop(Node):
    """Gripper-base teleop: analog joint + button-driven motor."""

    def __init__(self) -> None:
        super().__init__("gripper_base_teleop")

        # ── Parameters ──────────────────────────────────────────────────
        # Axis for the simple joint (M1)
        self.declare_parameter("axis_joint", 0)         # Left stick X

        # Buttons for the gripper motor (M2)
        self.declare_parameter("button_backward", 0)    # A button
        self.declare_parameter("button_forward", 1)     # B button

        # Speed scaling [0.0, 1.0]
        self.declare_parameter("speed", 0.5)

        # Deadzone for the analog axis
        self.declare_parameter("deadzone", 0.05)

        # Publish rate (Hz) — keeps RoboClaw watchdog alive
        self.declare_parameter("publish_hz", 20.0)

        # ── Read parameters ─────────────────────────────────────────────
        self.axis_joint = self.get_parameter("axis_joint").value
        self.button_backward = self.get_parameter("button_backward").value
        self.button_forward = self.get_parameter("button_forward").value
        self.speed = float(self.get_parameter("speed").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)

        # ── State ───────────────────────────────────────────────────────
        self._m1 = 0.0   # joint (analog)
        self._m2 = 0.0   # gripper (buttons)

        # ── Pub / Sub ──────────────────────────────────────────────────
        self.joy_sub = self.create_subscription(
            Joy, "joy", self._joy_cb, 10
        )
        self.out_pub = self.create_publisher(Joy, "gripper_base_joy", 10)

        # Periodic re-publish
        period = 1.0 / max(self.publish_hz, 1e-3)
        self.timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f"Gripper-base teleop ready  |  joint axis={self.axis_joint}  "
            f"btn_bwd={self.button_backward}  btn_fwd={self.button_forward}  "
            f"speed={self.speed:.0%}"
        )

    # ─────────────────────────────────────────────────────────────────────
    def _joy_cb(self, msg: Joy) -> None:
        """Read joystick and compute M1 / M2 commands."""

        # ── M1: analog joint from stick axis ────────────────────────────
        joint_raw = (
            msg.axes[self.axis_joint]
            if len(msg.axes) > self.axis_joint
            else 0.0
        )
        if abs(joint_raw) < self.deadzone:
            joint_raw = 0.0

        self._m1 = _clamp(joint_raw * self.speed)

        # ── M2: button-driven gripper ───────────────────────────────────
        btn_bwd = (
            msg.buttons[self.button_backward]
            if len(msg.buttons) > self.button_backward
            else 0
        )
        btn_fwd = (
            msg.buttons[self.button_forward]
            if len(msg.buttons) > self.button_forward
            else 0
        )

        # If both or neither pressed → stop; otherwise move in that direction
        if btn_fwd and not btn_bwd:
            self._m2 = self.speed
        elif btn_bwd and not btn_fwd:
            self._m2 = -self.speed
        else:
            self._m2 = 0.0

        # Publish immediately on input
        self._publish()

    def _publish(self) -> None:
        """Publish synthetic Joy for simple_roboclaw."""
        out = Joy()
        out.header.stamp = self.get_clock().now().to_msg()
        # axes[0] → simple_roboclaw axis_left  (M1 — joint)
        # axes[1] → simple_roboclaw axis_right (M2 — gripper)
        out.axes = [float(self._m1), float(self._m2)]
        self.out_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GripperBaseTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

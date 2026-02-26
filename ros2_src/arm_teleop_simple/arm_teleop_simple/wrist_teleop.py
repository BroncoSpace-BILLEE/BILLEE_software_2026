#!/usr/bin/env python3
"""
wrist_teleop.py

Differential wrist teleop using a directional pad (D-pad).

The robotic wrist has two motors driven by a single RoboClaw (M1 and M2).
The mechanism is a differential:
    - Both motors turning the SAME direction  → wrist pitch (up/down)
    - Motors turning in OPPOSITE directions    → wrist roll  (twist)

D-pad mapping (sensor_msgs/Joy axes):
    axis 7  (up/down)    :  +1 = up,   -1 = down   → pitch
    axis 6  (left/right) :  +1 = left, -1 = right   → roll (twist)

The node reads the real /joy topic, computes per-motor speeds from the
differential mixing, and publishes a synthetic sensor_msgs/Joy message
on /wrist/joy.  The simple_roboclaw node is launched with its joy topic
remapped to /wrist/joy so it picks up these commands directly.

Motor mixing:
    M1 = clamp(pitch + roll, -1, 1)   (left  motor)
    M2 = clamp(pitch - roll, -1, 1)   (right motor)

The output Joy message has:
    axes[0] = M1 speed (mapped to simple_roboclaw axis_left)
    axes[1] = M2 speed (mapped to simple_roboclaw axis_right)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


def _clamp(value: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, value))


class WristTeleop(Node):
    """Differential wrist teleop from D-pad."""

    def __init__(self) -> None:
        super().__init__("wrist_teleop")

        # ── Parameters ──────────────────────────────────────────────────
        # D-pad axis indices
        self.declare_parameter("axis_pitch", 7)   # D-pad up/down
        self.declare_parameter("axis_roll", 6)     # D-pad left/right

        # Speed scaling [0.0, 1.0] — fraction of max RoboClaw speed
        self.declare_parameter("speed", 0.5)

        # Deadzone (D-pad is typically ±1/0 so this is mostly a safety net)
        self.declare_parameter("deadzone", 0.05)

        # Output publish rate (Hz) — re-publishes the last command at this
        # rate so the RoboClaw watchdog doesn't time out while the D-pad
        # is held steady (which only sends one Joy message).
        self.declare_parameter("publish_hz", 20.0)

        # ── Read parameters ─────────────────────────────────────────────
        self.axis_pitch = self.get_parameter("axis_pitch").value
        self.axis_roll = self.get_parameter("axis_roll").value
        self.speed = float(self.get_parameter("speed").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)

        # ── State ───────────────────────────────────────────────────────
        self._m1 = 0.0
        self._m2 = 0.0

        # ── Pub / Sub ──────────────────────────────────────────────────
        self.joy_sub = self.create_subscription(
            Joy, "joy", self._joy_cb, 10
        )
        self.wrist_pub = self.create_publisher(Joy, "wrist_joy", 10)

        # Periodic re-publish so RoboClaw keeps receiving commands
        period = 1.0 / max(self.publish_hz, 1e-3)
        self.timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f"Wrist teleop ready  |  pitch axis={self.axis_pitch}  "
            f"roll axis={self.axis_roll}  speed={self.speed:.0%}"
        )

    # ─────────────────────────────────────────────────────────────────────
    def _joy_cb(self, msg: Joy) -> None:
        """Receive joystick, compute differential mix."""
        pitch_raw = (
            msg.axes[self.axis_pitch]
            if len(msg.axes) > self.axis_pitch
            else 0.0
        )
        roll_raw = (
            msg.axes[self.axis_roll]
            if len(msg.axes) > self.axis_roll
            else 0.0
        )

        # Apply deadzone
        if abs(pitch_raw) < self.deadzone:
            pitch_raw = 0.0
        if abs(roll_raw) < self.deadzone:
            roll_raw = 0.0

        # Scale by user speed parameter
        pitch = pitch_raw * self.speed
        roll = roll_raw * self.speed

        # Differential mixing
        #   Same direction  → pitch (up/down)
        #   Opposite        → roll  (twist)
        self._m1 = _clamp(pitch + roll)
        self._m2 = _clamp(pitch - roll)

        # Publish immediately so there's no latency on press
        self._publish()

    def _publish(self) -> None:
        """Publish synthetic Joy message for simple_roboclaw."""
        out = Joy()
        out.header.stamp = self.get_clock().now().to_msg()
        # axes[0] → mapped to simple_roboclaw axis_left  (M1)
        # axes[1] → mapped to simple_roboclaw axis_right (M2)
        out.axes = [float(self._m1), float(self._m2)]
        self.wrist_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = WristTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

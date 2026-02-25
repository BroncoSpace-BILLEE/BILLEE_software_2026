#!/usr/bin/env python3
"""
planar_ik_teleop.py

Simple planar 2-link inverse-kinematics velocity teleop.

Both joints rotate about the SAME axis (i.e. a planar RR arm in 2-D).
The joystick left-stick maps to desired end-effector Cartesian velocity
in the plane:
    - Left stick Y (axis 1)  →  forward / backward  (X in the arm plane)
    - Left stick X (axis 0)  →  up / down            (Y in the arm plane)

The node integrates joint angles internally, computes the 2×2 Jacobian,
inverts it, and publishes normalised joint-velocity commands as synthetic
sensor_msgs/Joy messages that two separate canopen_control nodes consume.

Coordinate convention (arm extends "forward" from the base):
    X-axis : horizontal, along the ground away from the base
    Y-axis : vertical, upward

    q1 is measured from the positive X-axis (0 = arm link-1 pointing forward)
    q2 is measured from link-1           (0 = link-2 collinear with link-1)

Publishes:
    /motor1/joy  (sensor_msgs/Joy)  – joint-1 velocity (normalised to [-1, 1])
    /motor2/joy  (sensor_msgs/Joy)  – joint-2 velocity (normalised to [-1, 1])

Subscribes:
    /joy         (sensor_msgs/Joy)  – real gamepad input
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


def _deadband(value: float, db: float) -> float:
    """Zero out values below the deadband threshold."""
    return 0.0 if abs(value) < db else value


def _clamp(value: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, value))


class PlanarIKTeleop(Node):
    """Planar 2-link IK velocity teleop node."""

    def __init__(self) -> None:
        super().__init__("planar_ik_teleop")

        # ── Parameters ──────────────────────────────────────────────────
        # Arm geometry
        self.declare_parameter("link1_length", 1.0)  # metres, base to joint-2
        self.declare_parameter("link2_length", 1.0)  # metres, joint-2 to end-effector

        # Initial joint angles (radians).  Default: elbow bent to avoid singularity.
        self.declare_parameter("q1_init", 1.2217)  # ~70° (link 1 droops 20° below vertical)
        self.declare_parameter("q2_init", -0.3491)  # ~-20° (link 2 bends 20° further down)

        # Maximum Cartesian velocity the joystick can request (m/s at full deflection)
        self.declare_parameter("max_cartesian_vel", 0.5)

        # Control rate (Hz)
        self.declare_parameter("publish_hz", 20.0)

        # Joystick
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("deadzone", 0.10)
        self.declare_parameter("joy_timeout_sec", 0.5)

        # Joystick axis mapping
        self.declare_parameter("joy_axis_x", 1)  # forward/backward (left stick Y)
        self.declare_parameter("joy_axis_y", 4)  # up/down (right stick Y)

        # Output topics (these will be remapped to per-motor joy topics)
        self.declare_parameter("motor1_joy_topic", "/motor1/joy")
        self.declare_parameter("motor2_joy_topic", "/motor2/joy")

        # Singularity avoidance: minimum Jacobian determinant magnitude
        self.declare_parameter("singularity_threshold", 0.01)

        # Joint angle limits (radians) – prevent the arm from folding back on itself
        self.declare_parameter("q1_min", -3.14159)
        self.declare_parameter("q1_max",  3.14159)
        self.declare_parameter("q2_min", -3.14159)
        self.declare_parameter("q2_max",  3.14159)

        # ── Read parameters ─────────────────────────────────────────────
        # Explicit float()/int() casts because launch-file LaunchConfiguration
        # values arrive as strings even when the default is numeric.
        self.L1 = float(self.get_parameter("link1_length").value)
        self.L2 = float(self.get_parameter("link2_length").value)
        self.q1 = float(self.get_parameter("q1_init").value)
        self.q2 = float(self.get_parameter("q2_init").value)
        self.max_cart_vel = float(self.get_parameter("max_cartesian_vel").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.joy_timeout = float(self.get_parameter("joy_timeout_sec").value)
        self.axis_x = int(self.get_parameter("joy_axis_x").value)
        self.axis_y = int(self.get_parameter("joy_axis_y").value)
        self.singularity_thresh = float(self.get_parameter("singularity_threshold").value)
        self.q1_min = float(self.get_parameter("q1_min").value)
        self.q1_max = float(self.get_parameter("q1_max").value)
        self.q2_min = float(self.get_parameter("q2_min").value)
        self.q2_max = float(self.get_parameter("q2_max").value)

        motor1_topic: str = self.get_parameter("motor1_joy_topic").value
        motor2_topic: str = self.get_parameter("motor2_joy_topic").value

        # ── State ───────────────────────────────────────────────────
        self.vx_cmd: float = 0.0  # desired Cartesian velocity X (forward)
        self.vy_cmd: float = 0.0  # desired Cartesian velocity Y (up)
        self.last_joy_time = None
        self._is_idle: bool = True   # track idle state to send zero only once

        # ── ROS interfaces ──────────────────────────────────────────────
        self.joy_sub = self.create_subscription(
            Joy, self.get_parameter("joy_topic").value, self._joy_cb, 10
        )
        self.motor1_pub = self.create_publisher(Joy, motor1_topic, 10)
        self.motor2_pub = self.create_publisher(Joy, motor2_topic, 10)

        dt = 1.0 / max(self.publish_hz, 1e-3)
        self.timer = self.create_timer(dt, self._control_loop)

        # ── Logging ─────────────────────────────────────────────────────
        self.get_logger().info(
            f"PlanarIKTeleop: L1={self.L1:.2f} m, L2={self.L2:.2f} m, "
            f"q1_init={math.degrees(self.q1):.1f}°, q2_init={math.degrees(self.q2):.1f}°"
        )
        self.get_logger().info(
            f"  Publishing motor cmds to: {motor1_topic}, {motor2_topic}"
        )
        fk_x, fk_y = self._fk()
        self.get_logger().info(
            f"  Initial end-effector position: ({fk_x:.3f}, {fk_y:.3f}) m"
        )

    # ── Forward kinematics ──────────────────────────────────────────────
    def _fk(self) -> tuple[float, float]:
        """Return (x, y) of the end-effector."""
        x = self.L1 * math.cos(self.q1) + self.L2 * math.cos(self.q1 + self.q2)
        y = self.L1 * math.sin(self.q1) + self.L2 * math.sin(self.q1 + self.q2)
        return x, y

    # ── Jacobian and inverse ────────────────────────────────────────────
    def _jacobian_inv(self) -> tuple[float, float, float, float] | None:
        """
        Compute the inverse of the 2×2 Jacobian:

            J = | -L1*s1 - L2*s12    -L2*s12 |
                |  L1*c1 + L2*c12     L2*c12 |

        Returns (inv_00, inv_01, inv_10, inv_11) or None if singular.
        """
        s1 = math.sin(self.q1)
        c1 = math.cos(self.q1)
        s12 = math.sin(self.q1 + self.q2)
        c12 = math.cos(self.q1 + self.q2)

        j11 = -self.L1 * s1 - self.L2 * s12
        j12 = -self.L2 * s12
        j21 = self.L1 * c1 + self.L2 * c12
        j22 = self.L2 * c12

        det = j11 * j22 - j12 * j21
        if abs(det) < self.singularity_thresh:
            return None

        inv_det = 1.0 / det
        return (
             j22 * inv_det,   # inv_00
            -j12 * inv_det,   # inv_01
            -j21 * inv_det,   # inv_10
             j11 * inv_det,   # inv_11
        )

    # ── Joystick callback ──────────────────────────────────────────────
    def _joy_cb(self, msg: Joy) -> None:
        axes = msg.axes
        if len(axes) <= max(self.axis_x, self.axis_y):
            return

        raw_x = _deadband(axes[self.axis_x], self.deadzone)
        raw_y = _deadband(axes[self.axis_y], self.deadzone)

        self.vx_cmd = _clamp(raw_x) * self.max_cart_vel
        self.vy_cmd = _clamp(raw_y) * self.max_cart_vel

        self.last_joy_time = self.get_clock().now()

    # ── Control loop ────────────────────────────────────────────────────
    def _control_loop(self) -> None:
        now = self.get_clock().now()
        dt = 1.0 / max(self.publish_hz, 1e-3)

        # Determine whether we should be idle (no joy data, stale, or zero cmd)
        should_idle = False
        if self.last_joy_time is None:
            should_idle = True
        else:
            age = (now - self.last_joy_time).nanoseconds * 1e-9
            if age > self.joy_timeout:
                should_idle = True

        vx = self.vx_cmd
        vy = self.vy_cmd
        if not should_idle and abs(vx) < 1e-6 and abs(vy) < 1e-6:
            should_idle = True

        # Send zero ONCE on transition to idle, then stop publishing
        if should_idle:
            if not self._is_idle:
                self._publish_zero()
                self._is_idle = True
            return

        self._is_idle = False

        # Inverse Jacobian
        inv = self._jacobian_inv()
        if inv is None:
            self.get_logger().warn(
                "Near singularity — cannot compute IK, sending zero velocity"
            )
            self._publish_zero()
            return

        inv_00, inv_01, inv_10, inv_11 = inv

        # Joint velocities (rad/s)
        q1_dot = inv_00 * vx + inv_01 * vy
        q2_dot = inv_10 * vx + inv_11 * vy

        # Integrate to update internal joint state
        q1_new = self.q1 + q1_dot * dt
        q2_new = self.q2 + q2_dot * dt

        # Clamp to joint limits
        q1_new = max(self.q1_min, min(self.q1_max, q1_new))
        q2_new = max(self.q2_min, min(self.q2_max, q2_new))

        # If clamped, zero out the corresponding velocity
        if q1_new == self.q1_min or q1_new == self.q1_max:
            q1_dot = 0.0
        if q2_new == self.q2_min or q2_new == self.q2_max:
            q2_dot = 0.0

        self.q1 = q1_new
        self.q2 = q2_new

        # Normalise joint velocities to [-1, 1] for the canopen_control nodes.
        # Each canopen_control node multiplies the axis value by max_velocity to
        # get the actual motor command in pulses/s, so we need to pick a
        # reasonable normalisation.  We normalise by the maximum joint velocity
        # that would occur at full Cartesian speed given the arm geometry.
        #
        # A conservative upper bound for the joint velocity magnitude is
        #   max_cart_vel / min(L1, L2)   (rad/s)
        # so we use that.
        max_qdot = self.max_cart_vel / min(self.L1, self.L2)
        if max_qdot < 1e-6:
            max_qdot = 1.0

        norm1 = _clamp(q1_dot / max_qdot)
        norm2 = _clamp(q2_dot / max_qdot)

        self._publish_motor_cmds(norm1, norm2)

        # Periodic debug log
        fk_x, fk_y = self._fk()
        self.get_logger().debug(
            f"vx={vx:.3f} vy={vy:.3f} | "
            f"q1_dot={q1_dot:.3f} q2_dot={q2_dot:.3f} | "
            f"norm=({norm1:.3f},{norm2:.3f}) | "
            f"q=({math.degrees(self.q1):.1f}°,{math.degrees(self.q2):.1f}°) | "
            f"EE=({fk_x:.3f},{fk_y:.3f})"
        )

    # ── Publishing helpers ──────────────────────────────────────────────
    def _make_joy_msg(self, axis_value: float) -> Joy:
        """
        Build a synthetic Joy message.

        canopen_control reads axes[joy_axis] — we set axis index 0 to the
        desired normalised value and leave the rest empty.  The launch file
        will set joy_axis=0 for each canopen_control instance.
        """
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [float(axis_value)]
        msg.buttons = []
        return msg

    def _publish_motor_cmds(self, norm1: float, norm2: float) -> None:
        self.motor1_pub.publish(self._make_joy_msg(norm1))
        self.motor2_pub.publish(self._make_joy_msg(norm2))

    def _publish_zero(self) -> None:
        self._publish_motor_cmds(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = PlanarIKTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

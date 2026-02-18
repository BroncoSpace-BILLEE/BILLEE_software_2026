#!/usr/bin/env python3
"""
send_gps_waypoints.py
─────────────────────
Converts GPS waypoints (lat/lon) → map-frame poses via robot_localization's
/fromLL service, then sends them to Nav2's /follow_waypoints action.

Usage
─────
  # Single waypoint:
  ros2 run chassis_bringup send_gps_waypoints.py --lat 45.123 --lon -73.456

  # Multiple waypoints from a YAML file:
  ros2 run chassis_bringup send_gps_waypoints.py --file waypoints.yaml

  waypoints.yaml format:
    waypoints:
      - {latitude: 45.123, longitude: -73.456}
      - {latitude: 45.124, longitude: -73.457}
"""

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoint
from nav2_msgs.action import FollowWaypoints
from robot_localization.srv import FromLL


class GpsWaypointSender(Node):
    def __init__(self):
        super().__init__('gps_waypoint_sender')
        self._fromll_cli = self.create_client(FromLL, '/fromLL')
        self._waypoint_cli = ActionClient(self, FollowWaypoints, '/follow_waypoints')

    # ── Convert a single GPS point to map-frame ──────────────────
    def gps_to_map(self, lat: float, lon: float, alt: float = 0.0):
        if not self._fromll_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/fromLL service not available')
            return None

        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=lat, longitude=lon, altitude=alt)
        future = self._fromll_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error('fromLL call failed')
            return None

        pt = future.result().map_point
        self.get_logger().info(
            f'GPS ({lat:.6f}, {lon:.6f}) → map ({pt.x:.3f}, {pt.y:.3f}, {pt.z:.3f})'
        )
        return pt

    # ── Send list of map-frame poses as waypoints ────────────────
    def send_waypoints(self, poses: list):
        if not self._waypoint_cli.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('/follow_waypoints action server not available')
            return

        goal = FollowWaypoints.Goal()
        goal.poses = poses

        self.get_logger().info(f'Sending {len(poses)} waypoint(s) …')
        send_future = self._waypoint_cli.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Waypoint goal REJECTED')
            return

        self.get_logger().info('Waypoint goal ACCEPTED — navigating …')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.missed_waypoints:
            self.get_logger().warn(
                f'Missed waypoints: {list(result.missed_waypoints)}'
            )
        else:
            self.get_logger().info('All waypoints reached!')

    def _feedback_cb(self, feedback_msg):
        self.get_logger().info(
            f'  ↳ heading to waypoint {feedback_msg.feedback.current_waypoint}'
        )


def main():
    parser = argparse.ArgumentParser(
        description='Send GPS waypoints to Nav2 via /fromLL + /follow_waypoints'
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        '--latlon', nargs='+', metavar='LAT,LON',
        help='One or more "lat,lon" pairs (e.g. --latlon 45.123,-73.456 45.124,-73.457)',
    )
    group.add_argument(
        '--file', type=str,
        help='YAML file with a "waypoints" list of {latitude, longitude} dicts',
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = GpsWaypointSender()

    # ── Parse waypoints ──────────────────────────────────────────
    gps_points = []
    if args.file:
        import yaml
        with open(args.file, 'r') as f:
            data = yaml.safe_load(f)
        for wp in data['waypoints']:
            gps_points.append((wp['latitude'], wp['longitude'], wp.get('altitude', 0.0)))
    else:
        for pair in args.latlon:
            parts = pair.split(',')
            if len(parts) < 2:
                node.get_logger().error(f'Bad format "{pair}" — expected LAT,LON')
                rclpy.shutdown()
                return 1
            gps_points.append((float(parts[0]), float(parts[1]),
                               float(parts[2]) if len(parts) > 2 else 0.0))

    # ── Convert & send ───────────────────────────────────────────
    poses = []
    for lat, lon, alt in gps_points:
        map_pt = node.gps_to_map(lat, lon, alt)
        if map_pt is None:
            node.get_logger().error('Aborting — GPS conversion failed')
            rclpy.shutdown()
            return 1

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x = map_pt.x
        pose.pose.position.y = map_pt.y
        pose.pose.position.z = map_pt.z
        pose.pose.orientation.w = 1.0
        poses.append(pose)

    node.send_waypoints(poses)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

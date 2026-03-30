#!/usr/bin/env python3
"""Converts Gazebo gpu_lidar LaserScan → Range (GUI) + scan_fixed (Nav2).

Gazebo Harmonic stamps LaserScan with scoped names like
``AgriBot/base_link/US1_sensor`` which don't exist in the ROS TF tree.
This node re-stamps with the real URDF frame ``US{N}_scan_link``
(created by the xacro) and republishes on ``/ultra/us{N}/scan_fixed``
so that Nav2 costmaps and collision_monitor can resolve transforms.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range


class UltrasonicConverter(Node):
    # Maps sensor key → correct URDF TF frame (from xacro)
    SENSORS = {
        'us1': 'US1_scan_link',
        'us2': 'US2_scan_link',
        'us3': 'US3_scan_link',
        'us4': 'US4_scan_link',
    }

    def __init__(self):
        super().__init__('ultrasonic_converter')
        self.range_pubs = {}
        self.scan_pubs = {}

        for key, frame in self.SENSORS.items():
            self.range_pubs[key] = self.create_publisher(Range, f'/ultra/{key}', 10)
            self.scan_pubs[key] = self.create_publisher(LaserScan, f'/ultra/{key}/scan_fixed', 10)
            self.create_subscription(
                LaserScan, f'/ultra/{key}/scan',
                lambda msg, s=key, f=frame: self._cb(msg, s, f),
                qos_profile_sensor_data,
            )

        self.get_logger().info('Ultrasonic converter ready.')

    # ------------------------------------------------------------------
    def _cb(self, msg: LaserScan, sensor_key: str, static_frame: str):
        now = self.get_clock().now().to_msg()

        # Re-stamp scan with correct URDF frame for Nav2
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = static_frame
        scan.angle_min = msg.angle_min
        scan.angle_max = msg.angle_max
        scan.angle_increment = msg.angle_increment
        scan.time_increment = msg.time_increment
        scan.scan_time = msg.scan_time
        scan.range_min = max(msg.range_min, 0.02)
        scan.range_max = msg.range_max
        scan.ranges = list(msg.ranges)
        scan.intensities = list(msg.intensities) if msg.intensities else []
        self.scan_pubs[sensor_key].publish(scan)

        # Min range → Range msg for GUI ultrasonics display
        valid = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        dist = min(valid) if valid else float('inf')

        r = Range()
        r.header.stamp = now
        r.header.frame_id = static_frame
        r.radiation_type = Range.ULTRASOUND
        r.field_of_view = abs(msg.angle_max - msg.angle_min) if msg.angle_max != msg.angle_min else 1.047
        r.min_range = msg.range_min
        r.max_range = msg.range_max
        r.range = dist
        self.range_pubs[sensor_key].publish(r)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(UltrasonicConverter())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Broadcasts a static identity transform  map → odom.

This replaces AMCL / map_server for the case where we assume
the EKF-fused odometry is our only localisation source."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class StaticMapTf(Node):
    def __init__(self):
        super().__init__('static_map_tf')
        broadcaster = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0  # identity quaternion
        broadcaster.sendTransform(t)

        self.get_logger().info('Published static map → odom identity TF.')


def main(args=None):
    rclpy.init(args=args)
    node = StaticMapTf()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

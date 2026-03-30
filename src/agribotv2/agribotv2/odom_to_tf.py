#!/usr/bin/env python3
"""
Odometry to TF Publisher

Subscribes to ground truth odometry from Gazebo and publishes the 
odom->base_link transform to /tf.

This is needed because Gazebo Ignition's internal TF system doesn't 
bridge cleanly to ROS 2. Instead, we read the perfect odometry message
and republish it as a TF transform.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.msg_count = 0
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('===========================================')
        self.get_logger().info('Odometry to TF publisher started')
        self.get_logger().info('Subscribing to: /odom')
        self.get_logger().info('Publishing TF: odom -> base_link')
        self.get_logger().info('===========================================')

    def odom_callback(self, msg):
        """Convert odometry message to TF transform"""
        self.msg_count += 1
        if self.msg_count % 30 == 0:  # Log every 30 messages (every second at 30Hz)
            self.get_logger().info(f'Publishing TF from odom data (message #{self.msg_count})')
        
        t = TransformStamped()
        
        # Header
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id  # Should be 'odom'
        t.child_frame_id = msg.child_frame_id    # Should be 'base_link'
        
        # Translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Rotation
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        
        # Broadcast
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

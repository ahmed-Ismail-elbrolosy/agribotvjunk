#!/usr/bin/env python3
"""
Ground Truth Odometry Publisher
Reads robot's actual pose from Gazebo physics engine (not wheel odometry!)
and publishes as odometry + TF.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
import math

class GroundTruthOdom(Node):
    def __init__(self):
        super().__init__('ground_truth_odom')
        
        # Subscribe to Gazebo's world dynamic pose info (contains all model poses)
        self.create_subscription(
            TFMessage,
            '/world/SinaiAgri/dynamic_pose/info',
            self.tf_callback,
            10
        )
        
        # Publish corrected odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Ground Truth Odometry Node Started (using world dynamic_pose/info)')
    
    def tf_callback(self, msg):
        """
        Parse TF message from Gazebo and extract AgriBot's pose
        """
        origin_tf = None
        robot_tf = None
        for transform in msg.transforms:
            if transform.child_frame_id.endswith('origin_marker') or 'origin_marker' in transform.child_frame_id:
                origin_tf = transform.transform
            if transform.child_frame_id == 'AgriBot' or 'AgriBot' in transform.child_frame_id:
                robot_tf = transform.transform

        if origin_tf is None or robot_tf is None:
            return

        # Compute pose of robot relative to origin_marker
        rel_trans, rel_rot = self.relative_transform(origin_tf, robot_tf)

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'origin_marker'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = rel_trans[0]
        odom.pose.pose.position.y = rel_trans[1]
        odom.pose.pose.position.z = rel_trans[2]
        odom.pose.pose.orientation.x = rel_rot[0]
        odom.pose.pose.orientation.y = rel_rot[1]
        odom.pose.pose.orientation.z = rel_rot[2]
        odom.pose.pose.orientation.w = rel_rot[3]

        # Publish odometry
        self.odom_pub.publish(odom)

        # Publish TF transform origin_marker -> base_link
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'origin_marker'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = rel_trans[0]
        t.transform.translation.y = rel_trans[1]
        t.transform.translation.z = rel_trans[2]
        t.transform.rotation.x = rel_rot[0]
        t.transform.rotation.y = rel_rot[1]
        t.transform.rotation.z = rel_rot[2]
        t.transform.rotation.w = rel_rot[3]

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def quat_inverse(q):
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if norm == 0:
            return (0.0, 0.0, 0.0, 1.0)
        return (-q.x/norm, -q.y/norm, -q.z/norm, q.w/norm)

    @staticmethod
    def quat_multiply(q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        )

    def relative_transform(self, origin_tf, robot_tf):
        # Translation difference
        dx = robot_tf.translation.x - origin_tf.translation.x
        dy = robot_tf.translation.y - origin_tf.translation.y
        dz = robot_tf.translation.z - origin_tf.translation.z

        # Rotate difference by inverse of origin orientation
        q0 = origin_tf.rotation
        q0_inv = self.quat_inverse(q0)
        # Represent translation as quaternion (v,0)
        trans_q = (dx, dy, dz, 0.0)
        rot_q = self.quat_multiply(self.quat_multiply(q0_inv, trans_q), (q0.x, q0.y, q0.z, q0.w))
        rel_trans = (rot_q[0], rot_q[1], rot_q[2])

        # Relative orientation: q_rel = q0_inv * q_robot
        q_robot = (robot_tf.rotation.x, robot_tf.rotation.y, robot_tf.rotation.z, robot_tf.rotation.w)
        q_rel = self.quat_multiply(q0_inv, q_robot)

        return rel_trans, q_rel

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

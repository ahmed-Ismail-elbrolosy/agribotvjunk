"""
ground_truth_odom — Extracts the robot's true world pose from
Gazebo's /world/<world>/dynamic_pose/info (bridged as TFMessage
on /gz_dynamic_poses) and publishes nav_msgs/Odometry on /odom_gt.

This gives the EKF a ground-truth odometry source that perfectly
matches where the robot actually is in Gazebo.
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class GroundTruthOdom(Node):
    MODEL_NAME = 'AgriBot'

    def __init__(self):
        super().__init__('ground_truth_odom')
        self.pub = self.create_publisher(Odometry, '/odom_gt', 10)
        self.sub = self.create_subscription(
            TFMessage, '/gz_dynamic_poses', self._cb, 10
        )
        self.prev_time = None
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        self.get_logger().info('Ground truth odom node started, listening on /gz_dynamic_poses')

    def _cb(self, msg: TFMessage):
        if not msg.transforms:
            return

        # The first transform in dynamic_pose/info is always the model
        # root (AgriBot).  The Pose_V→TFMessage bridge loses the name
        # field but preserves ordering.
        tf = msg.transforms[0]

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        t = tf.transform.translation
        r = tf.transform.rotation
        odom.pose.pose.position.x = t.x
        odom.pose.pose.position.y = t.y
        odom.pose.pose.position.z = t.z
        odom.pose.pose.orientation = r

        # Estimate velocity from consecutive poses
        cur_time = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_time is not None:
            dt = cur_time - self.prev_time
            if dt > 0.0:
                import math
                # Extract yaw from quaternion
                siny_cosp = 2.0 * (r.w * r.z + r.x * r.y)
                cosy_cosp = 1.0 - 2.0 * (r.y * r.y + r.z * r.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)

                dx = t.x - self.prev_x
                dy = t.y - self.prev_y

                # Body-frame velocities
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                odom.twist.twist.linear.x = cos_yaw * dx / dt + sin_yaw * dy / dt
                odom.twist.twist.linear.y = -sin_yaw * dx / dt + cos_yaw * dy / dt
                if self.prev_yaw is not None:
                    dyaw = yaw - self.prev_yaw
                    # Normalize angle
                    while dyaw > math.pi:
                        dyaw -= 2.0 * math.pi
                    while dyaw < -math.pi:
                        dyaw += 2.0 * math.pi
                    odom.twist.twist.angular.z = dyaw / dt
                self.prev_yaw = yaw

        self.prev_time = cur_time
        self.prev_x = t.x
        self.prev_y = t.y

        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

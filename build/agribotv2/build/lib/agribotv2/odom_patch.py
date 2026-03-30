import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomPatch(Node):
    def __init__(self):
        super().__init__('odom_patch')
        
        # Subscribe to the raw (zero covariance) odom from Gazebo
        self.sub = self.create_subscription(
            Odometry,
            '/odom_raw',
            self.odom_callback,
            10
        )
        
        # Publish the fixed (with covariance) odom to /odometry/fixed
        self.pub = self.create_publisher(Odometry, '/odometry/fixed', 10)
        
        # --- POSE COVARIANCE (Position) ---
        # 0.001 = Very high confidence
        # 0.1   = Medium confidence
        # 100.0 = Do not trust (High uncertainty)
        # Sequence: x, y, z, roll, pitch, yaw
        self.pose_covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # X (Trust wheel encoders)
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,  # Y (Trust wheel encoders)
            0.0, 0.0, 100.0, 0.0, 0.0, 0.0, # Z (Don't trust wheel odom for height)
            0.0, 0.0, 0.0, 100.0, 0.0, 0.0, # Roll (Don't trust)
            0.0, 0.0, 0.0, 0.0, 100.0, 0.0, # Pitch (Don't trust)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01   # Yaw (Trust encoders for turning)
        ]

        # --- TWIST COVARIANCE (Velocity) ---
        # Gazebo actually gives very accurate velocity, so we can trust this more
        self.twist_covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # Vx
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,  # Vy
            0.0, 0.0, 1e-6, 0.0, 0.0, 0.0,  # Vz (Should be 0)
            0.0, 0.0, 0.0, 1e-6, 0.0, 0.0,  # V_Roll
            0.0, 0.0, 0.0, 0.0, 1e-6, 0.0,  # V_Pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01   # V_Yaw
        ]

    def odom_callback(self, msg):
        # Create a new message to ensure we don't modify the incoming pointer improperly
        new_msg = Odometry()
        new_msg.header = msg.header
        new_msg.child_frame_id = msg.child_frame_id
        
        # Copy the data
        new_msg.pose.pose = msg.pose.pose
        new_msg.twist.twist = msg.twist.twist
        
        # Inject the covariances
        new_msg.pose.covariance = self.pose_covariance
        new_msg.twist.covariance = self.twist_covariance
        
        # Explicitly ensure frame IDs are correct if Gazebo messes them up
        # Standard: odom -> base_link
        new_msg.header.frame_id = 'odom'
        new_msg.child_frame_id = 'base_link'
        
        # Publish
        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPatch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
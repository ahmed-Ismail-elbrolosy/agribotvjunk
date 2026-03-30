#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math

class UltrasonicAvoidanceNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_avoidance_node')

        # --- CONFIGURATION ---
        # Task 2: Explicit topic names (no wildcards)
        self.declare_parameter('ultrasonic_topics', [
            '/ultra/us1', 
            '/ultra/us2', 
            '/ultra/us3', 
            '/ultra/us4'
        ])
        
        # Task 4 & 5: Constraints
        self.declare_parameter('view_radius', 1.0)      # meters
        self.declare_parameter('safety_distance', 0.40) # meters
        self.declare_parameter('max_steering_rad', 1.0) # max angular velocity correction
        self.declare_parameter('base_frame', 'base_link')

        # Load parameters
        self.us_topics = self.get_parameter('ultrasonic_topics').value
        self.view_radius = self.get_parameter('view_radius').value
        self.safety_dist = self.get_parameter('safety_distance').value
        self.max_steer = self.get_parameter('max_steering_rad').value
        self.base_frame = self.get_parameter('base_frame').value

        # --- STATE ---
        # Store latest range keyed by frame_id: {'us_link_1': 0.5, ...}
        self.latest_ranges = {}
        # Store current command from nav/teleop
        self.current_cmd = Twist()

        # --- TF SETUP (Task 3) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- SUBSCRIBERS (Task 2) ---
        self.subs = []
        for topic in self.us_topics:
            self.get_logger().info(f'Subscribing to: {topic}')
            # Use partial or lambda to capture logic if needed, 
            # but generic callback works because we use msg.header.frame_id
            sub = self.create_subscription(
                LaserScan,
                topic,
                self.range_callback,
                qos_profile_sensor_data # Best effort for sensors
            )
            self.subs.append(sub)

        # Input command subscriber (Task 1)
        self.create_subscription(Twist, 'cmd_vel_in', self.cmd_in_callback, 10)

        # --- PUBLISHER (Task 7) ---
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # --- CONTROL LOOP ---
        # Run at 20Hz to process latest data and TF
        self.create_timer(0.05, self.control_loop)

    def range_callback(self, msg):
        """Task 2: Store data using frame_id as key."""
        # Task 4: Pre-filter invalid readings immediately
        # For LaserScan, we take the minimum range
        if not msg.ranges:
            return
            
        r = min(msg.ranges)
        
        if r <= 0 or math.isnan(r) or math.isinf(r):
            # If sensor reports garbage, remove from active consideration or ignore
            if msg.header.frame_id in self.latest_ranges:
                del self.latest_ranges[msg.header.frame_id]
            return

        # Store valid range
        self.latest_ranges[msg.header.frame_id] = r

    def cmd_in_callback(self, msg):
        """Store the incoming command from Nav2/Teleop."""
        self.current_cmd = msg

    def control_loop(self):
        """Core logic loop: TF lookup, Safety Check, Steering Calc."""
        
        emergency_stop = False
        steering_correction = 0.0
        
        # We need to process every sensor we have data for
        # Create a list of keys to iterate safely
        frame_ids = list(self.latest_ranges.keys())

        for frame_id in frame_ids:
            dist = self.latest_ranges[frame_id]

            # Task 4: Perception Window (Ignore > 1.0m)
            if dist > self.view_radius:
                continue

            # Task 3: Geometry Extraction via TF
            try:
                # Lookup transform: base_link -> ultrasonic_frame
                # We need the most recent transform available
                t = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    frame_id,
                    rclpy.time.Time())
                
                # Extract lateral position (y) relative to robot center
                y_pos = t.transform.translation.y
                
            except (LookupException, ConnectivityException, ExtrapolationException):
                # If TF fails, we cannot use this sensor for logic this cycle
                continue

            # Task 5: Safety Constraint
            if dist < self.safety_dist:
                emergency_stop = True
                self.get_logger().warn(f'EMERGENCY: {frame_id} detected obstacle at {dist:.2f}m')
                # We continue the loop to calculate steering, but the flag is set

            # Task 6: Steering Calculation (Continuous)
            # 1. Distance Weight: Linear decay (Close = 1.0, Far = 0.0)
            w_dist = 1.0 - (dist / self.view_radius)
            
            # 2. Center Weight: Inverse lateral distance
            # Epsilon added to prevent division by zero if sensor is exactly centered
            epsilon = 0.1
            w_center = 1.0 / (abs(y_pos) + epsilon)

            # 3. Total Force
            force = w_dist * w_center

            # 4. Direction Logic
            # Robot Frame: Y+ is Left, Y- is Right.
            # Rotation: Z+ is Left (CCW), Z- is Right (CW).
            # If Obstacle on Left (y > 0) -> We want to turn Right (Z-) -> subtract force
            # If Obstacle on Right (y < 0) -> We want to turn Left (Z+) -> add force
            
            if y_pos > 0:
                steering_correction -= force
            else:
                steering_correction += force

        # Task 6: Clamp Steering
        steering_correction = max(min(steering_correction, self.max_steer), -self.max_steer)

        # Task 7: Output Command
        out_msg = Twist()
        
        if emergency_stop:
            # Task 5: Stop forward velocity
            out_msg.linear.x = 0.0
            # We still allow rotation to try and turn away from the danger
            out_msg.angular.z = steering_correction
        else:
            # Pass through original velocity
            out_msg.linear.x = self.current_cmd.linear.x
            # Add reactive steering to original steering
            out_msg.angular.z = self.current_cmd.angular.z + steering_correction

        # Task 9: Validation - Ensure symmetric cancellation
        # If forces cancel out perfectly, steering_correction is near 0.
        # If emergency_stop is True (both close), linear.x becomes 0.
        
        self.cmd_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

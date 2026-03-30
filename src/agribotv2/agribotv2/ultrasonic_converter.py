import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import random

class UltrasonicConverter(Node):
    def __init__(self):
        super().__init__('ultrasonic_converter')
        
        # 1. Map topics to the STATIC URDF links
        # Ensure these names match your sensors.xacro EXACTLY
        self.sensor_map = {
            'us1': 'US1_scan_link',
            'us2': 'US2_scan_link',
            'us3': 'US3_scan_link',
            'us4': 'US4_scan_link'
        }
        
        self.range_pubs = {}
        self.scan_pubs = {}
        self.tf_broadcaster = TransformBroadcaster(self)
        self.spin_angle = 0.0

        for topic_suffix, static_frame in self.sensor_map.items():
            # Range Publisher
            self.range_pubs[topic_suffix] = self.create_publisher(Range, f'/ultra/{topic_suffix}', 10)
            # Scan Publisher
            self.scan_pubs[topic_suffix] = self.create_publisher(LaserScan, f'/ultra/{topic_suffix}/scan_fixed', 10)
            
            # Subscriber
            self.create_subscription(
                LaserScan,
                f'/ultra/{topic_suffix}/scan',
                lambda msg, s=topic_suffix, f=static_frame: self.scan_callback(msg, s, f),
                qos_profile_sensor_data 
            )
            
        self.get_logger().info('Ultrasonic Converter: TF Broadcasting Active.')

    def get_quaternion_from_roll(self, roll):
        qx = math.sin(roll / 2)
        qy = 0.0
        qz = 0.0
        qw = math.cos(roll / 2)
        return (qx, qy, qz, qw)

    def scan_callback(self, msg, sensor_key, static_frame_id):
        current_time = self.get_clock().now().to_msg()
        
        # --- 1. SPIN LOGIC ---
        self.spin_angle += 0.6 
        if self.spin_angle > 6.283:
            self.spin_angle = 0.0
            
        # --- 2. BROADCAST TF (The Link #3) ---
        spinning_frame_id = f"{sensor_key}_spinning_link"
        
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = static_frame_id # Parent (Must exist in URDF!)
        t.child_frame_id = spinning_frame_id
        
        (qx, qy, qz, qw) = self.get_quaternion_from_roll(self.spin_angle)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(t)

        # --- 3. PROCESS DATA ---
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        current_range = float('inf')
        if valid_ranges:
            current_range = min(valid_ranges)

        # --- 4. PUBLISH SPINNING SCAN ---
        fake_scan = LaserScan()
        fake_scan.header.stamp = current_time # MUST match TF time
        fake_scan.header.frame_id = spinning_frame_id
        
        fake_scan.angle_min = -0.5235 
        fake_scan.angle_max = 0.5235  
        
        # Density settings
        num_readings = 100 
        fake_scan.angle_increment = (fake_scan.angle_max - fake_scan.angle_min) / num_readings
        fake_scan.time_increment = 0.0
        fake_scan.scan_time = 0.0
        fake_scan.range_min = 0.2
        fake_scan.range_max = 4.0
        
        new_ranges = []
        
        for i in range(num_readings):
            if current_range == float('inf'):
                new_ranges.append(float('inf'))
            else:
                if random.random() > 0.9: 
                    new_ranges.append(float('inf'))
                else:
                    noise = random.uniform(-0.02, 0.02)
                    new_ranges.append(current_range + noise)

        fake_scan.ranges = new_ranges
        self.scan_pubs[sensor_key].publish(fake_scan)
        
        # --- 5. PUBLISH RANGE (CONE) ---
        range_msg = Range()
        range_msg.header.stamp = current_time
        range_msg.header.frame_id = static_frame_id
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 1.047
        range_msg.min_range = msg.range_min
        range_msg.max_range = msg.range_max
        range_msg.range = current_range
            
        self.range_pubs[sensor_key].publish(range_msg)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
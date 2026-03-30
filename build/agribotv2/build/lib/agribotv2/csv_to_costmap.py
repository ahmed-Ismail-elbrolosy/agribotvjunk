#!/usr/bin/env python3
import os
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct

def create_cloud_xyz32(header, points):
    """
    Create a sensor_msgs/PointCloud2 from a list of (x, y, z) points.
    """
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.is_dense = False
    msg.is_bigendian = False
    
    # Define XYZ fields
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    ]
    
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    
    # Pack data
    buffer = bytearray()
    for p in points:
        buffer.extend(struct.pack('<fff', p[0], p[1], p[2]))
        
    msg.data = bytes(buffer)
    return msg

class CSVToCostmapNode(Node):
    def __init__(self):
        super().__init__('csv_to_costmap')
        
        self.declare_parameter('csv_path', '')
        # Determine CSV path
        csv_path_param = self.get_parameter('csv_path').value
        if csv_path_param:
            self.csv_file = csv_path_param
        else:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('agribotv2')
            self.csv_file = os.path.join(pkg_share, 'GUI', 'agribot_data', 'grid_cells.csv')
            
        self.get_logger().info(f"Monitoring CSV: {self.csv_file}")
        
        # Publisher for the Plant PointCloud
        self.publisher = self.create_publisher(PointCloud2, '/detected_plants', 10)
        
        # Timer to read CSV periodically (every 1 second)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.last_mtime = 0

    def timer_callback(self):
        if not os.path.exists(self.csv_file):
            return

        current_mtime = os.path.getmtime(self.csv_file)
        # To ensure we continually re-publish the cloud for costmap persistence,
        # we don't strictly gate on mtime, or we publish regularly. 
        # The costmap might need continuous clearing/marking.
        
        points = []
        try:
            with open(self.csv_file, mode='r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    state = row.get('state', '')
                    data = row.get('data', '')
                    
                    if state.lower() == 'planted' and data:
                        coords = data.split(',')
                        if len(coords) == 2:
                            try:
                                x = float(coords[0])
                                y = float(coords[1])
                                # Add point (add a slight z to ensure standard 3D points)
                                points.append((x, y, 0.2))  # z=0.2 represents obstacle height
                            except ValueError:
                                pass
        except Exception as e:
            self.get_logger().error(f"Error reading CSV: {e}")
            return
            
        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # Grid cells coordinates are typically in world/map frame
        
        pc2_msg = create_cloud_xyz32(header, points)
        self.publisher.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CSVToCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Reads grid_cells.csv, publishes rows flagged as 'planted' as a PointCloud2
so Nav2 costmaps treat them as obstacles."""

import csv
import struct
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class CSVCostmapNode(Node):
    def __init__(self):
        super().__init__('csv_costmap_node')

        self.declare_parameter('csv_path', '')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('point_height', 0.25)
        self.declare_parameter('cell_width', 0.4)
        self.declare_parameter('cell_height', 0.4)
        self.declare_parameter('point_spacing', 0.1)

        csv_path = self.get_parameter('csv_path').value
        if not csv_path:
            # default: <pkg>/data/grid_cells.csv  (installed location)
            pkg_share = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'share', 'agribotv3', 'data',
            )
            csv_path = os.path.join(pkg_share, 'grid_cells.csv')

        self.csv_path = csv_path
        self.point_height = self.get_parameter('point_height').value
        self.cell_width = self.get_parameter('cell_width').value
        self.cell_height = self.get_parameter('cell_height').value
        self.point_spacing = self.get_parameter('point_spacing').value
        rate = self.get_parameter('publish_rate').value

        self.pub = self.create_publisher(PointCloud2, '/detected_plants', 10)
        self.timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(f'CSV costmap node → {self.csv_path}')

    # ------------------------------------------------------------------
    def _publish(self):
        points = self._read_csv()
        if not points:
            return
        self.pub.publish(self._make_cloud(points))

    def _read_csv(self) -> list[tuple[float, float]]:
        if not os.path.isfile(self.csv_path):
            return []
        pts = []
        cw = self.cell_width
        ch = self.cell_height
        sp = self.point_spacing
        try:
            with open(self.csv_path, newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if row.get('status', '').strip().lower() != 'planted':
                        continue
                    data = row.get('data', '')
                    try:
                        cx, cy = (float(v) for v in data.split(',')[:2])
                        # Publish a dense grid of points within the cell
                        # so the costmap treats the entire cell as an obstacle
                        dx = -cw / 2 + sp / 2
                        while dx < cw / 2:
                            dy = -ch / 2 + sp / 2
                            while dy < ch / 2:
                                pts.append((cx + dx, cy + dy))
                                dy += sp
                            dx += sp
                    except (ValueError, IndexError):
                        pass
        except Exception as e:
            self.get_logger().warn(f'CSV read error: {e}')
        return pts

    def _make_cloud(self, pts: list[tuple[float, float]]) -> PointCloud2:
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        data = bytearray()
        for x, y in pts:
            data += struct.pack('fff', x, y, self.point_height)

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = len(pts)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.data = bytes(data)
        msg.is_dense = True
        return msg


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CSVCostmapNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

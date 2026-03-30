#!/usr/bin/env python3
"""Vision node – detects green plants via HSV masking, estimates distance,
optionally optimises a path through detected plants using scipy spline fitting."""

import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray, Point
from std_msgs.msg import String, Bool


# ---------------------------------------------------------------------------
# Plant detector (pure-vision, no ROS)
# ---------------------------------------------------------------------------
class PlantDetector:
    """HSV green-mask → contours → distance estimation."""

    def __init__(self):
        self.hsv_lower = np.array([35, 40, 40])
        self.hsv_upper = np.array([85, 255, 255])
        self.focal_length = 554.0
        self.real_plant_height = 0.3  # metres

    def detect(self, bgr: np.ndarray):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        plants = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w // 2
            cy = y + h // 2
            dist = (self.focal_length * self.real_plant_height) / max(h, 1)
            plants.append({
                'bbox': (x, y, w, h),
                'centre': (cx, cy),
                'area': area,
                'distance': dist,
            })
        return plants


# ---------------------------------------------------------------------------
# Scipy-based path optimiser (optional – only used when triggered)
# ---------------------------------------------------------------------------
class PathOptimiser:
    """Smooth a waypoint list using cubic spline + Douglas-Peucker."""

    def __init__(self, smoothing: float = 0.5, simplify_eps: float = 0.1):
        self.smoothing = smoothing
        self.simplify_eps = simplify_eps

    def optimise(self, waypoints: list[tuple[float, float]],
                 obstacles: list[tuple[float, float]] | None = None):
        if len(waypoints) < 3:
            return waypoints
        try:
            from scipy.interpolate import splprep, splev
        except ImportError:
            return waypoints

        pts = np.array(waypoints, dtype=np.float64)
        tck, _ = splprep([pts[:, 0], pts[:, 1]], s=self.smoothing, k=3)
        u_fine = np.linspace(0, 1, max(len(waypoints) * 10, 100))
        smooth = np.column_stack(splev(u_fine, tck))

        # Douglas-Peucker simplification
        simplified = [smooth[0]]
        self._rdp(smooth.tolist(), self.simplify_eps, simplified)
        simplified.append(smooth[-1].tolist())
        return [(p[0], p[1]) for p in simplified]

    # ---- recursive Douglas-Peucker ----
    @staticmethod
    def _rdp(pts, eps, result):
        if len(pts) < 3:
            return
        dmax, idx = 0.0, 0
        p1, p2 = np.array(pts[0]), np.array(pts[-1])
        line = p2 - p1
        line_len = np.linalg.norm(line)
        if line_len == 0:
            return
        for i in range(1, len(pts) - 1):
            d = abs(np.cross(line, p1 - np.array(pts[i]))) / line_len
            if d > dmax:
                dmax, idx = d, i
        if dmax > eps:
            PathOptimiser._rdp(pts[:idx + 1], eps, result)
            result.append(pts[idx])
            PathOptimiser._rdp(pts[idx:], eps, result)


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------
class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.detector = PlantDetector()
        self.optimiser = PathOptimiser()
        self.obstacles: list[tuple[float, float]] = []
        self.robot_pose: Pose | None = None

        # Parameters
        self.declare_parameter('hsv_lower', [35, 40, 40])
        self.declare_parameter('hsv_upper', [85, 255, 255])
        self.declare_parameter('min_contour_area', 500)
        self.declare_parameter('focal_length', 554.0)
        self.declare_parameter('real_plant_height', 0.3)

        self._apply_params()

        # Subscribers
        self.create_subscription(Image, '/camera/image_raw', self._image_cb, qos_profile_sensor_data)
        self.create_subscription(Pose, '/vision/robot_pose', self._pose_cb, 10)
        self.create_subscription(PoseArray, '/vision/optimize_path', self._optimize_cb, 10)
        self.create_subscription(Point, '/vision/add_obstacle', self._add_obs_cb, 10)
        self.create_subscription(Bool, '/vision/clear_obstacles', self._clear_obs_cb, 10)

        # Publishers
        self.det_pub = self.create_publisher(String, '/vision/plant_detections', 10)
        self.path_pub = self.create_publisher(PoseArray, '/vision/optimized_path', 10)
        self.status_pub = self.create_publisher(String, '/vision/status', 10)

        self.get_logger().info('Vision node ready.')

    # ---- helpers ----
    def _apply_params(self):
        lo = self.get_parameter('hsv_lower').value
        hi = self.get_parameter('hsv_upper').value
        self.detector.hsv_lower = np.array(lo, dtype=np.uint8)
        self.detector.hsv_upper = np.array(hi, dtype=np.uint8)
        self.detector.focal_length = self.get_parameter('focal_length').value
        self.detector.real_plant_height = self.get_parameter('real_plant_height').value

    @staticmethod
    def _ros_image_to_bgr(msg: Image) -> np.ndarray:
        if msg.encoding in ('bgr8', 'bgra8'):
            channels = 4 if msg.encoding == 'bgra8' else 3
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)[:, :, :3]
        if msg.encoding in ('rgb8', 'rgba8'):
            channels = 4 if msg.encoding == 'rgba8' else 3
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)[:, :, :3]
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return np.zeros((msg.height, msg.width, 3), dtype=np.uint8)

    # ---- callbacks ----
    def _image_cb(self, msg: Image):
        bgr = self._ros_image_to_bgr(msg)
        plants = self.detector.detect(bgr)
        if plants:
            import json
            det_data = []
            for p in plants:
                det_data.append({
                    'cx': p['centre'][0],
                    'cy': p['centre'][1],
                    'area': p['area'],
                    'distance': round(p['distance'], 3),
                })
            out = String()
            out.data = json.dumps(det_data)
            self.det_pub.publish(out)

        status = String()
        status.data = f'detected {len(plants)} plants'
        self.status_pub.publish(status)

    def _pose_cb(self, msg: Pose):
        self.robot_pose = msg

    def _optimize_cb(self, msg: PoseArray):
        waypoints = [(p.position.x, p.position.y) for p in msg.poses]
        optimised = self.optimiser.optimise(waypoints, self.obstacles or None)
        out = PoseArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        for x, y in optimised:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            out.poses.append(p)
        self.path_pub.publish(out)

    def _add_obs_cb(self, msg: Point):
        self.obstacles.append((msg.x, msg.y))
        self.get_logger().info(f'Added obstacle at ({msg.x:.2f}, {msg.y:.2f}), total={len(self.obstacles)}')

    def _clear_obs_cb(self, _msg: Bool):
        self.obstacles.clear()
        self.get_logger().info('Cleared all obstacles.')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VisionNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

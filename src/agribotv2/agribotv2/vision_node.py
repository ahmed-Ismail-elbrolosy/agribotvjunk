#!/usr/bin/env python3
"""
Vision and Path Optimization Node for AgriBot.
UPDATED: Fixes KeyError: 0 by handling Dictionary detections correctly.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import json
import math
import traceback

# Optional imports with fallbacks
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("[VisionNode] OpenCV not available - vision detection disabled")

try:
    from scipy import optimize
    from scipy.interpolate import splprep, splev
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("[VisionNode] SciPy not available - using basic path optimization")


class PlantDetector:
    def __init__(self):
        self.IMG_W = 640  # Standardize your camera res (e.g. 640x480)
        self.IMG_H = 480
        
        # Camera Field of View (Horizontal)
        # Standard webcam is ~60 degrees (1.047 radians)
        # Wide angle is ~90 degrees (1.57 radians)
        self.FOV_H = 1.047 
        
        # Focal length calculation (pixels)
        # f = (width / 2) / tan(fov / 2)
        self.FOCAL_LENGTH = (self.IMG_W / 2) / math.tan(self.FOV_H / 2)

        # ROI: Look at the ground in front
        # Don't look at the very bottom (bumper), look slightly up
        self.roi_y1 = int(self.IMG_H * 0.4) 
        self.roi_y2 = int(self.IMG_H * 0.9)
        
        # Physical mounting offset (Camera position relative to robot center)
        self.CAM_OFFSET_X = 0.45  # Camera is 0.45m in front of robot center
        
        # Green Mask Settings
        self.hsv_lower = np.array([30, 40, 40])
        self.hsv_upper = np.array([90, 255, 255])

    def detect(self, image, robot_x, robot_y, robot_yaw):
        if not CV2_AVAILABLE: return []

        # Resize for consistent math
        img = cv2.resize(image, (self.IMG_W, self.IMG_H))
        
        # Crop to ROI
        roi = img[self.roi_y1:self.roi_y2, 0:self.IMG_W]
        
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        
        # Clean noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        
        for c in contours:
            area = cv2.contourArea(c)
            if area < 500: continue 
            
            # 1. Find Centroid
            M = cv2.moments(c)
            if M["m00"] == 0: continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            full_cy = cy + self.roi_y1
            
            # 2. Calculate Distance (Lower in image = Closer)
            # Map normalized Y (0.0 to 1.0) to physical distance (0.5m to 2.5m)
            norm_y = 1.0 - (full_cy / self.IMG_H) 
            dist_from_cam = 0.5 + (norm_y * 2.0)
            
            # 3. Calculate Angle offset
            # Center of image is angle 0
            offset_px = cx - (self.IMG_W / 2)
            
            # --- FIX IS HERE ---
            # We multiply by -1 because Image Right is Robot Negative Angle (Clockwise)
            angle_offset = -1 * math.atan2(offset_px, self.FOCAL_LENGTH)
            
            # 4. Calculate World Coordinates
            total_yaw = robot_yaw + angle_offset
            
            # Cam position in world
            cam_wx = robot_x + self.CAM_OFFSET_X * math.cos(robot_yaw)
            cam_wy = robot_y + self.CAM_OFFSET_X * math.sin(robot_yaw)
            
            # Object position
            obj_x = cam_wx + dist_from_cam * math.cos(total_yaw)
            obj_y = cam_wy + dist_from_cam * math.sin(total_yaw)
            
            detections.append({
                'x': obj_x,
                'y': obj_y,
                'type': 'plant',
                'conf': 1.0
            })

        return detections


class ScipyPathOptimizer:
    """Path optimization using SciPy (Unchanged)."""
    def __init__(self, resolution=0.15, robot_radius=0.6):
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.obstacles = []
    
    def add_obstacle(self, x: float, y: float, radius: float):
        self.obstacles.append((x, y, radius))
    
    def clear_obstacles(self):
        self.obstacles = []
    
    def smooth_path(self, waypoints: list, smoothness: float = 0.5) -> list:
        if not SCIPY_AVAILABLE or len(waypoints) < 3: return waypoints
        try:
            points = np.array(waypoints)
            k = min(3, len(waypoints) - 1)
            s = smoothness * len(waypoints)
            tck, u = splprep([points[:, 0], points[:, 1]], s=s, k=k)
            u_new = np.linspace(0, 1, max(len(waypoints) * 3, 20))
            x_new, y_new = splev(u_new, tck)
            return list(zip(x_new.tolist(), y_new.tolist()))
        except Exception: return waypoints
    
    def optimize_path_length(self, waypoints: list, iterations: int = 100) -> list:
        if not SCIPY_AVAILABLE or len(waypoints) < 3: return waypoints
        points = np.array(waypoints)
        start, end = points[0], points[-1]
        x0 = points[1:-1].flatten()
        
        def cost_function(x):
            pts = x.reshape(-1, 2)
            full_path = np.vstack([start, pts, end])
            diffs = np.diff(full_path, axis=0)
            length = np.sum(np.sqrt(np.sum(diffs**2, axis=1)))
            penalty = 0.0
            for ox, oy, r in self.obstacles:
                for p in pts:
                    dist = np.sqrt((p[0] - ox)**2 + (p[1] - oy)**2)
                    if dist < r + self.robot_radius:
                        penalty += 1000 * (r + self.robot_radius - dist)
            return length + penalty
        
        try:
            result = optimize.minimize(cost_function, x0, method='L-BFGS-B', options={'maxiter': iterations, 'disp': False})
            optimized = result.x.reshape(-1, 2)
            full_path = np.vstack([start, optimized, end])
            return [tuple(p) for p in full_path]
        except Exception: return waypoints
    
    def douglas_peucker(self, points: list, epsilon: float = 0.1) -> list:
        if len(points) < 3: return points
        start, end = np.array(points[0]), np.array(points[-1])
        dmax, index = 0, 0
        line_vec = end - start
        if np.linalg.norm(line_vec) < 1e-10: return [points[0], points[-1]]
        line_unit = line_vec / np.linalg.norm(line_vec)
        for i in range(1, len(points) - 1):
            pt = np.array(points[i])
            proj = start + np.dot(pt - start, line_unit) * line_unit
            d = np.linalg.norm(pt - proj)
            if d > dmax: index, dmax = i, d
        if dmax > epsilon:
            return self.douglas_peucker(points[:index+1], epsilon)[:-1] + self.douglas_peucker(points[index:], epsilon)
        return [points[0], points[-1]]


class VisionPathNode(Node):
    """ROS 2 Node for vision detection and path optimization."""
    
    def __init__(self):
        super().__init__('vision_path_node')
        
        self.detector = PlantDetector() if CV2_AVAILABLE else None
        self.path_optimizer = ScipyPathOptimizer() if SCIPY_AVAILABLE else None
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Subscribers
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(String, '/vision/robot_pose', self.pose_callback, 10)
        self.create_subscription(String, '/vision/optimize_path', self.optimize_path_callback, 10)
        self.create_subscription(String, '/vision/add_obstacle', self.add_obstacle_callback, 10)
        self.create_subscription(String, '/vision/clear_obstacles', self.clear_obstacles_callback, 10)
        
        # Publishers
        self.plant_pub = self.create_publisher(String, '/vision/plant_detections', 10)
        self.optimized_path_pub = self.create_publisher(String, '/vision/optimized_path', 10)
        self.status_pub = self.create_publisher(String, '/vision/status', 10)
        
        self.get_logger().info(f"VisionPathNode initialized")
    
    def pose_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.robot_x = data.get('x', 0.0)
            self.robot_y = data.get('y', 0.0)
            self.robot_yaw = data.get('yaw', 0.0)
        except json.JSONDecodeError:
            pass
    
    def image_callback(self, msg: Image):
        """Process camera image for plant detection."""
        if not self.detector: return
        
        try:
            if msg.encoding == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            else: return
            
            # --- DETECT ---
            # Returns a list of dictionaries: [{'x':..., 'type':...}]
            detections = self.detector.detect(img, self.robot_x, self.robot_y, self.robot_yaw)
            
            if detections:
                # --- PUBLISH ---
                # FIX: Directly dump the detections list (it is already in correct format)
                # DO NOT iterate and use d[0]
                result = String()
                result.data = json.dumps(detections)
                self.plant_pub.publish(result)
                
        except Exception:
            self.get_logger().error(f"Image processing error:\n{traceback.format_exc()}")
    
    def add_obstacle_callback(self, msg: String):
        if not self.path_optimizer: return
        try:
            data = json.loads(msg.data)
            self.path_optimizer.add_obstacle(data.get('x', 0), data.get('y', 0), data.get('radius', 0.3))
        except: pass
    
    def clear_obstacles_callback(self, msg: String):
        if self.path_optimizer: self.path_optimizer.clear_obstacles()
    
    def optimize_path_callback(self, msg: String):
        if not self.path_optimizer:
            self.optimized_path_pub.publish(msg)
            return
        try:
            data = json.loads(msg.data)
            waypoints = [(p['x'], p['y']) for p in data.get('waypoints', [])]
            smooth = data.get('smooth', True)
            opt_len = data.get('optimize', True)
            
            result = waypoints
            if opt_len and len(waypoints) > 2:
                result = self.path_optimizer.optimize_path_length(result)
            if smooth:
                result = self.path_optimizer.smooth_path(result, smoothness=0.3)
            result = self.path_optimizer.douglas_peucker(result, epsilon=0.05)
            
            response = String()
            response.data = json.dumps({'waypoints': [{'x': p[0], 'y': p[1]} for p in result]})
            self.optimized_path_pub.publish(response)
        except Exception:
            self.optimized_path_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
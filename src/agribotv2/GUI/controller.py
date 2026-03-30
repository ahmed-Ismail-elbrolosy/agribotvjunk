import sys
import math
import time
import random
import json
import numpy as np
from datetime import datetime

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image, Range
from std_msgs.msg import String

# PyQt5 Imports
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, QMutex
from PyQt5.QtGui import QImage

# Local Imports
import config
from astar import AStarPlanner
from fast_planner import FastPlanner, ObstacleAvoidanceController

# Try OpenCV (optional - for advanced detection visualization)
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

class RosController(QThread):
    # Signals
    update_odom = pyqtSignal(float, float, float)
    update_cam = pyqtSignal(QImage)
    update_path_viz = pyqtSignal(dict, list) 
    update_seeder = pyqtSignal(str, float) 
    update_sensors = pyqtSignal(dict) 
    point_reached = pyqtSignal(float, float, str) 
    job_finished = pyqtSignal(str) 
    soil_scan_results = []  # Collect soil scan data during job
    log_msg = pyqtSignal(str)
    show_popup = pyqtSignal(str, str)
    plant_detected = pyqtSignal()  # Signal to refresh map when plant is detected

    def __init__(self):
        super().__init__()
        self.node = None
        self.mutex = QMutex()
        
        self.global_waypoints = []
        self.local_path = []
        self.current_goal = None
        self.state = "IDLE" 
        self.current_job_type = None 
        
        self.planner = FastPlanner(
            config.WORLD_MIN_X, config.WORLD_MAX_X, 
            config.WORLD_MIN_Y, config.WORLD_MAX_Y, 
            resolution=0.15, robot_radius=0.6, min_turn_radius=0.75
        )
        
        # Smart obstacle avoidance controller
        self.obstacle_avoider = ObstacleAvoidanceController()
        
        self.us_ranges = {'us1': 4.0, 'us2': 4.0, 'us3': 4.0, 'us4': 4.0}
        self.reversing = False  # Flag for reverse maneuver
        self.reverse_start_time = 0
        self.last_replan_time = 0  # Cooldown for path replanning
        self.replan_cooldown = 1.0  # Reduced from 2.0 for faster response
        self.awaiting_replan = False
        self.last_replan_attempt = 0
        self.last_progress_pos = (0.0, 0.0)
        self.last_front_pos = (0.0, 0.0)
        self.last_progress_check_time = time.time()
        self.last_head_dist = None
        self.last_head_check_time = time.time()
        self.known_obstacles = []  # Track registered obstacles to avoid duplicates
        self.avoiding_obstacle = False  # Flag for obstacle avoidance mode
        self.db_ref = None 
        self.seeder_state = "IDLE"
        self.seeder_height = 0.20 
        self.seeder_timer = 0
        self.manual_vx = 0.0; self.manual_vw = 0.0; self.manual_seeder_vel = 0.0 
        self.rx = 0.0; self.ry = 0.0; self.ryaw = 0.0
        self.selected_seed = None
        self.selected_seed_depth_m = 0.10  # default 10cm
        self.kp_linear = 2.0; self.kd_linear = 1.5; self.kp_angular = 1.8; self.kd_angular = 0.5; 
        self.max_linear = 1.8; self.max_angular = 1.6; self.prev_angular_error = 0.0; self.last_time = 0
        
        self.IMG_W = 640; self.IMG_H = 480
        self.FOV_H = 1.047 
        self.FOCAL_LENGTH = (self.IMG_W / 2) / math.tan(self.FOV_H / 2) 
        self.REAL_PLANT_WIDTH = 0.20 
        self.CAM_OFF_X = 0.45; self.CAM_OFF_Y = -0.0334
        
        # Grid parameters for coordinate conversion
        self.grid_min_x = config.GRID_CONFIG['min_x']
        self.grid_min_y = config.GRID_CONFIG['min_y']
        self.grid_cols = config.GRID_CONFIG['cols']
        self.grid_rows = config.GRID_CONFIG['rows']
        self.cell_w = config.CELL_W
        self.cell_h = config.CELL_H

    def world_to_grid(self, wx, wy):
        """Convert world coordinates to grid (col, row)."""
        # Column is Left -> Right (Standard)
        col = int((wx - self.grid_min_x) / self.cell_w)
        
        # Row is Top -> Bottom (To match MapWidget/Matrix logic)
        # We assume Max Y is Min Y + (Rows * Height)
        grid_max_y = self.grid_min_y + (self.grid_rows * self.cell_h)
        row = int((grid_max_y - wy) / self.cell_h)
        
        return col, row
    
    def grid_to_world(self, col, row):
        """Convert grid (col, row) to world center coordinates."""
        wx = self.grid_min_x + (col + 0.5) * self.cell_w
        wy = self.grid_min_y + (row + 0.5) * self.cell_h
        return wx, wy

    def get_plant_recommendation(self, soil_data):
        """
        Placeholder function for plant recommendation based on soil analysis.
        Currently randomly picks from seeds table - will be replaced with ML/rule-based system.
        
        Args:
            soil_data: dict with keys 'n', 'p', 'k', 'ph', 'moisture', 'temp', 'ec'
        
        Returns:
            tuple: (seed_name, planting_depth_cm, reason) or (None, None, "No seeds available")
        """
        if not self.db_ref:
            return (None, None, "Database not available")
        
        seeds = self.db_ref.get_all_seeds()
        if not seeds:
            return (None, None, "No seeds in database")
        
        # TODO: Replace with actual recommendation logic based on soil_data
        # For now, randomly pick a seed as placeholder
        selected = random.choice(seeds)
        seed_name = selected[1]
        planting_depth = selected[2]
        
        # Placeholder reason - future: analyze soil_data to determine best match
        reason = f"Placeholder recommendation (random selection)"
        
        return (seed_name, planting_depth, reason)

    def _show_soil_scan_summary(self):
        """Generate and show summary report of all soil scans from the job."""
        if not self.soil_scan_results:
            return
        
        n_scans = len(self.soil_scan_results)
        
        # Calculate averages
        avg_n = sum(r['n'] for r in self.soil_scan_results) / n_scans
        avg_p = sum(r['p'] for r in self.soil_scan_results) / n_scans
        avg_k = sum(r['k'] for r in self.soil_scan_results) / n_scans
        avg_ph = sum(r['ph'] for r in self.soil_scan_results) / n_scans
        avg_moisture = sum(r['moisture'] for r in self.soil_scan_results) / n_scans
        avg_temp = sum(r['temp'] for r in self.soil_scan_results) / n_scans
        avg_ec = sum(r['ec'] for r in self.soil_scan_results) / n_scans
        
        # Get plant recommendation based on average soil data
        avg_soil = {
            'n': avg_n, 'p': avg_p, 'k': avg_k,
            'ph': avg_ph, 'moisture': avg_moisture,
            'temp': avg_temp, 'ec': avg_ec
        }
        rec_seed, rec_depth, rec_reason = self.get_plant_recommendation(avg_soil)
        
        # Build summary message
        summary = f"=== SOIL SCAN SUMMARY ===\n"
        summary += f"Scans completed: {n_scans}\n\n"
        summary += f"--- Average Values ---\n"
        summary += f"Nitrogen (N): {avg_n:.1f} ppm\n"
        summary += f"Phosphorus (P): {avg_p:.1f} ppm\n"
        summary += f"Potassium (K): {avg_k:.1f} ppm\n"
        summary += f"pH: {avg_ph:.1f}\n"
        summary += f"Moisture: {avg_moisture:.1f}%\n"
        summary += f"Temperature: {avg_temp:.1f}°C\n"
        summary += f"EC: {avg_ec:.2f} mS/cm\n\n"
        
        if rec_seed:
            summary += f"--- Recommendation ---\n"
            summary += f"Suggested Plant: {rec_seed}\n"
            summary += f"Planting Depth: {rec_depth:.1f} cm\n"
            summary += f"Note: {rec_reason}"
        
        # Show popup with summary
        self.show_popup.emit("Soil Scan Report", summary)
        
        # Clear results for next job
        self.soil_scan_results = []

    def run(self):
        if not rclpy.ok(): rclpy.init()
        self.node = Node('agribot_controller_node')
        
        # Log CV2 status
        if CV2_AVAILABLE:
            self.log_msg.emit("Vision System: Local OpenCV available - Plant detection ACTIVE")
        else:
            self.log_msg.emit("Vision System: Waiting for external vision node...")
        
        self.pub_vel = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Vision node communication publishers
        self.pub_robot_pose = self.node.create_publisher(String, '/vision/robot_pose', 10)
        self.pub_optimize_path = self.node.create_publisher(String, '/vision/optimize_path', 10)
        self.pub_add_obstacle = self.node.create_publisher(String, '/vision/add_obstacle', 10)
        self.pub_clear_obstacles = self.node.create_publisher(String, '/vision/clear_obstacles', 10)
        
        # Vision node communication subscribers
        self.node.create_subscription(String, '/vision/plant_detections', self.vision_detection_cb, 10)
        self.node.create_subscription(String, '/vision/optimized_path', self.optimized_path_cb, 10)
        self.node.create_subscription(String, '/vision/status', self.vision_status_cb, 10)
        
        self.node.create_subscription(TFMessage, '/world/SinaiAgri/dynamic_pose/info', self.tf_cb, 10)
        self.node.create_subscription(Image, '/camera/image_raw', self.cam_cb, 10)
        for sensor in ['us1', 'us2', 'us3', 'us4']:
            self.node.create_subscription(Range, f'/ultra/{sensor}', lambda msg, s=sensor: self.us_cb(msg, s), 10)

        self.last_time = time.time()
        self.timer = self.node.create_timer(0.05, self.control_loop) 
        try: rclpy.spin(self.node)
        except Exception as e: self.log_msg.emit(f"ROS Error: {e}")
        finally: self.stop_robot(); self.node.destroy_node(); rclpy.shutdown()

    def vision_status_cb(self, msg):
        """Receive status from external vision node."""
        try:
            status = json.loads(msg.data)
            cv2_avail = status.get('cv2_available', False)
            scipy_avail = status.get('scipy_available', False)
            np_ver = status.get('numpy_version', 'unknown')
            self.log_msg.emit(f"Vision Node: CV2={cv2_avail}, SciPy={scipy_avail}, NumPy={np_ver}")
        except json.JSONDecodeError:
            pass
    def vision_detection_cb(self, msg):
        """
        Handles vision data.
        Updates map, adds obstacles, and REMOVES planted points from job queue.
        """
        try:
            detections = json.loads(msg.data)
            robot_x = self.rx
            robot_y = self.ry
            
            # Tolerance for matching a detection to a waypoint (meters)
            MATCH_TOLERANCE = 0.4 

            for d in detections:
                look_at_x = d['x']
                look_at_y = d['y']
                det_type = d.get('type', 'plant')
                
                # 1. Coordinate Conversion (using corrected col, row order)
                col, row = self.world_to_grid(look_at_x, look_at_y)
                
                # Bounds check
                if not (0 <= col < self.grid_cols and 0 <= row < self.grid_rows):
                    continue
                
                cell_center_x, cell_center_y = self.grid_to_world(col, row)
                
                # Get current status from DB
                current_status = None
                if self.db_ref:
                    current_status = self.db_ref.get_cell_status(col, row)

                # --- LOGIC BRANCHING ---

                if det_type == 'plant':
                    # A) UPDATE MAP & OBSTACLES
                    if current_status not in ['planted', 'obstacle', 'detected']:
                        if self.db_ref:
                            self.db_ref.update_cell(col, row, 'detected')
                        
                        # FIX 1: Effective Avoidance
                        # Increase radius to 0.4m. 
                        # Robot Radius (0.6) + Plant Buffer (0.4) = 1.0m separation center-to-center.
                        self.add_and_validate_obstacle(cell_center_x, cell_center_y, radius=0.4)
                        self.log_msg.emit(f"Vision: Found NEW PLANT at Grid[{col},{row}]")
                        
                        # Emit signal to redraw map
                        self.plant_detected.emit()

                    # B) CHECK CURRENT GOAL (Drop if already planted)
                    # If we are currently driving to this point to plant it, STOP.
                    if self.current_goal and self.current_goal.get('type') in ['plant', 'plant_scan']:
                        dist_to_goal = math.hypot(self.current_goal['x'] - look_at_x, self.current_goal['y'] - look_at_y)
                        
                        if dist_to_goal < MATCH_TOLERANCE:
                            self.log_msg.emit(f"Skipping Goal: Point ({look_at_x:.1f}, {look_at_y:.1f}) is already planted!")
                            
                            # Mark as planted in DB so we don't come back
                            if self.db_ref:
                                self.db_ref.update_cell(col, row, 'detected') # or 'planted'
                            
                            # Force State Transition
                            self.mutex.lock()
                            self.state = "NEXT_GLOBAL_POINT"
                            self.current_goal = None
                            self.local_path = [] # Clear path
                            self.stop_robot()
                            self.mutex.unlock()
                            continue # Skip rest of loop

                    # C) CHECK FUTURE WAYPOINTS (Drop from queue)
                    # Filter out any future planting points that match this detection
                    if self.global_waypoints:
                        initial_count = len(self.global_waypoints)
                        
                        # Keep points that are EITHER not planting tasks OR are far away from this plant
                        self.global_waypoints = [
                            wp for wp in self.global_waypoints 
                            if not (
                                wp.get('type') in ['plant', 'plant_scan'] and 
                                math.hypot(wp['x'] - look_at_x, wp['y'] - look_at_y) < MATCH_TOLERANCE
                            )
                        ]
                        
                        removed_count = initial_count - len(self.global_waypoints)
                        if removed_count > 0:
                            self.log_msg.emit(f"Optimization: Removed {removed_count} future waypoints (already planted)")
                            # Update map visualization to show the path changed
                            self.update_path_viz.emit(self.current_goal or {}, self.global_waypoints)

                elif det_type == 'free':
                    # Verify Empty Ground
                    if current_status in ['empty', None, '']:
                        if self.db_ref:
                            self.db_ref.update_cell(col, row, 'checked')
                    
                    # Missing Plant Logic
                    elif current_status == 'planted':
                        dist_to_cell = math.sqrt((robot_x - cell_center_x)**2 + (robot_y - cell_center_y)**2)
                        # Constraint: Only remove if we are VERY close (0.8m) to be sure
                        if dist_to_cell < 0.8:
                            self.log_msg.emit(f"Vision: Missing plant at [{col},{row}]")
                            if self.db_ref:
                                self.db_ref.update_cell(col, row, 'empty')

        except Exception as e:
            self.log_msg.emit(f"Vision Callback Error: {e}")
            import traceback
            print(traceback.format_exc())    
    
    def optimized_path_cb(self, msg):
        """Receive optimized path from external vision node."""
        try:
            data = json.loads(msg.data)
            waypoints = data.get('waypoints', [])
            if waypoints:
                self.local_path = waypoints
                self.log_msg.emit(f"Received optimized path with {len(waypoints)} points")
        except json.JSONDecodeError:
            pass
    
    def publish_robot_pose(self):
        """Send current robot pose to vision node."""
        if hasattr(self, 'pub_robot_pose'):
            msg = String()
            msg.data = json.dumps({'x': self.rx, 'y': self.ry, 'yaw': self.ryaw})
            self.pub_robot_pose.publish(msg)
    
    def request_path_optimization(self, path):
        """Request path optimization from vision node."""
        if hasattr(self, 'pub_optimize_path'):
            msg = String()
            msg.data = json.dumps({
                'waypoints': [{'x': p['x'], 'y': p['y']} for p in path],
                'smooth': True,
                'optimize': True
            })
            self.pub_optimize_path.publish(msg)

    def us_cb(self, msg, sensor_id):
        r = min(msg.range, config.US_MAX_RANGE)
        self.us_ranges[sensor_id] = r
        self.update_sensors.emit(self.us_ranges)
        
        # Register obstacles for path planning
        # Front sensors (us2, us3): detect obstacles within 2.5m
        if sensor_id in ['us2', 'us3'] and r < 2.5:
            self.register_obstacle_at_dist(sensor_id, r)
        # Side sensors (us1 right, us4 left): only register if very close
        if sensor_id in ['us1', 'us4'] and r < 0.3:
            self.register_obstacle_at_dist(sensor_id, r)
        
        # Path replanning trigger: stop and wait for fresh path instead of rotating in place
        if self.state == "FOLLOW_PATH" and sensor_id in ['us2', 'us3']:
            front_min = min(self.us_ranges['us2'], self.us_ranges['us3'])
            if front_min < 1.5:
                self.stop_robot()
                self.awaiting_replan = True
                self.last_replan_attempt = 0
                self.state = "WAIT_REPLAN"
                self.log_msg.emit(f"OBSTACLE DETECTED at {front_min:.2f}m - waiting for replan")

    def trigger_reverse(self, sensor_id, dist):
        """Trigger reverse maneuver when front sensor detects obstacle < 0.8m"""
        if not self.reversing and self.state != "REVERSING":
            self.reversing = True
            self.reverse_start_time = time.time()
            self.prev_state = self.state
            self.state = "REVERSING"
            self.log_msg.emit(f"REVERSE: {sensor_id} obstacle at {dist:.2f}m - backing up to 1.0m")

    def register_obstacle_at_dist(self, sensor, dist):
        off = config.SENSOR_OFFSETS[sensor]
        # Transform sensor offset to world frame
        cos_yaw = math.cos(self.ryaw)
        sin_yaw = math.sin(self.ryaw)
        sensor_wx = self.rx + (off['x'] * cos_yaw - off['y'] * sin_yaw)
        sensor_wy = self.ry + (off['x'] * sin_yaw + off['y'] * cos_yaw)
        # Sensor's global yaw
        global_s_yaw = self.ryaw + off['yaw']
        # Obstacle position
        obs_x = sensor_wx + (dist * math.cos(global_s_yaw))
        obs_y = sensor_wy + (dist * math.sin(global_s_yaw))
        
        # Check if this obstacle is already known (within 0.3m of existing)
        for known in self.known_obstacles:
            if math.hypot(obs_x - known[0], obs_y - known[1]) < 0.3:
                return  # Already registered, skip
        
        # Register new obstacle
        self.known_obstacles.append((obs_x, obs_y, time.time()))
        # Keep only recent obstacles (last 30 seconds)
        self.known_obstacles = [(x, y, t) for x, y, t in self.known_obstacles if time.time() - t < 30]
        
        self.add_and_validate_obstacle(obs_x, obs_y)
        self.log_msg.emit(f"OBSTACLE: {sensor} d={dist:.2f}m @ ({obs_x:.2f},{obs_y:.2f})")
    def add_and_validate_obstacle(self, obs_x, obs_y, radius=1.1):
        """
        Registers an obstacle with the planner.
        Args:
            obs_x, obs_y: World coordinates
            radius: The radius of the obstacle (default 1.1m for safety, 0.15m for plants)
        """
        # Use the passed radius instead of hardcoding 1.1
        self.planner.add_obstacle(obs_x, obs_y, radius=radius)
        
        if self.state == "FOLLOW_PATH" and self.local_path:
            # Check cooldown - don't replan too frequently
            if time.time() - self.last_replan_time < self.replan_cooldown:
                return
            
            is_blocked = False
            check_nodes = self.local_path[:15]
            
            # Check if path is too close to this new obstacle
            # We add a small buffer (0.1) to the radius for the check
            safety_dist = radius + 0.1
            
            for pt in check_nodes:
                d = math.sqrt((pt['x'] - obs_x)**2 + (pt['y'] - obs_y)**2)
                if d < safety_dist: 
                    is_blocked = True
                    break 
            
            if is_blocked:
                self.last_replan_time = time.time()
                new_path = self.planner.plan(self.rx, self.ry, self.current_goal['x'], self.current_goal['y'], robot_yaw=self.ryaw)
                if new_path: 
                    self.local_path = new_path
                    self.log_msg.emit(f"PATH REPLANNED around new obstacle")
                else: 
                    self.stop_robot()
                    self.log_msg.emit("Error: Path Blocked by new detection")
    def tf_cb(self, msg):
        for transform in msg.transforms:
            if 'AgriBot' in transform.child_frame_id or transform.child_frame_id == 'AgriBot':
                self.rx = transform.transform.translation.x
                self.ry = transform.transform.translation.y
                q = transform.transform.rotation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y); cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                self.ryaw = math.atan2(siny_cosp, cosy_cosp)
                self.update_odom.emit(self.rx, self.ry, self.ryaw)
                # Publish pose to vision node
                self.publish_robot_pose()
                break

    def cam_cb(self, msg):
        try:
            # Direct ROS Image to numpy conversion (no cv_bridge needed)
            img = self._ros_image_to_numpy(msg)
            if img is None:
                return
            
            # --- VISION REDUNDANCY REMOVED ---
            # We no longer process CV internal to the controller.
            # The dedicated 'vision_node.py' handles this and sends detections via ROS.
            # This keeps the GUI lightweight and stable.
            
            # Convert to QImage for display
            h, w = img.shape[:2]
            if len(img.shape) == 3:
                ch = img.shape[2]
                # Convert BGR to RGB for Qt and ensure contiguous
                rgb_img = np.ascontiguousarray(img[:, :, ::-1])
                bytes_per_line = ch * w
                qt_img = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888).copy()
            else:
                rgb_img = np.ascontiguousarray(img)
                bytes_per_line = w
                qt_img = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_Grayscale8).copy()
            
            self.update_cam.emit(qt_img)
        except Exception as e:
            self.log_msg.emit(f"Camera Error: {str(e)}")

    def _ros_image_to_numpy(self, msg):
        """Convert ROS Image message to numpy array without cv_bridge"""
        # Supported encodings
        encoding = msg.encoding
        h, w = msg.height, msg.width
        
        if encoding in ['rgb8', 'bgr8']:
            dtype = np.uint8
            channels = 3
        elif encoding in ['rgba8', 'bgra8']:
            dtype = np.uint8
            channels = 4
        elif encoding in ['mono8', '8UC1']:
            dtype = np.uint8
            channels = 1
        elif encoding in ['mono16', '16UC1']:
            dtype = np.uint16
            channels = 1
        else:
            # Try to handle as RGB8
            dtype = np.uint8
            channels = 3
        
        # Convert to numpy
        img = np.frombuffer(msg.data, dtype=dtype)
        
        if channels == 1:
            img = img.reshape((h, w))
        else:
            img = img.reshape((h, w, channels))
        
        # Convert RGB to BGR if needed (OpenCV uses BGR)
        if encoding == 'rgb8':
            img = img[:, :, ::-1].copy()
        elif encoding == 'rgba8':
            img = img[:, :, [2, 1, 0, 3]].copy()
        
        return img

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def set_manual_vel(self, vx, vw):
        self.mutex.lock(); self.state = "MANUAL" if vx!=0 or vw!=0 else "IDLE"; self.manual_vx = vx; self.manual_vw = vw; self.mutex.unlock()

    def set_manual_seeder(self, vel):
        self.mutex.lock(); self.state = "MANUAL"; self.manual_seeder_vel = vel; self.mutex.unlock()

    def set_path(self, points, job_type="MOVE"):
        self.mutex.lock(); self.state = "IDLE"; self.current_goal = None; self.local_path = []

        # Filter out waypoints that correspond to already-planted/detected cells
        filtered = []
        if self.db_ref:
            for pt in points:
                if pt.get('type') in ['plant', 'plant_scan']:
                    c = int((pt['x'] - config.GRID_CONFIG['min_x']) / config.CELL_W)
                    r = int((config.GRID_CONFIG['max_y'] - pt['y']) / config.CELL_H)
                    state = self.db_ref.get_cell_status(r, c)
                    if state in ['planted', 'detected', 'obstacle']:
                        continue
                filtered.append(pt)
        else:
            filtered = points

        self.global_waypoints = filtered; self.current_job_type = job_type
        self.soil_scan_results = []  # Clear previous soil scan results for new job
        self.planner.clear_obstacles()
        self.known_obstacles = []  # Clear tracked obstacles for new path
        self.last_replan_time = 0  # Reset replan cooldown
        self.avoiding_obstacle = False  # Reset obstacle avoidance
        self.awaiting_replan = False
        self.obstacle_avoider.reset()

        # Track number of planting targets for completion popup
        self.plant_targets_count = sum(1 for pt in self.global_waypoints if pt.get('type') in ['plant', 'plant_scan'])

        # Load default seed selection/depth from DB
        if self.db_ref:
            seeds = self.db_ref.get_all_seeds()
            if seeds:
                self.selected_seed = seeds[0][1]
                self.selected_seed_depth_m = max(0.02, min(0.20, seeds[0][2] / 100.0))  # clamp 2-20cm

        # Target cells for this job (so we don't mark them as obstacles)
        target_cells = set()
        for pt in points:
            tc = int((pt['x'] - config.GRID_CONFIG['min_x']) / config.CELL_W)
            tr = int((config.GRID_CONFIG['max_y'] - pt['y']) / config.CELL_H)
            target_cells.add((tc, tr))
        if self.db_ref:
            cells = self.db_ref.get_all_cells()
            for cell in cells:
                if cell[2] in ['planted', 'detected']:  # Treat both planted and detected as obstacles
                    if (cell[0], cell[1]) in target_cells:
                        continue  # allow reaching the target cell even if flagged
                    # Check if we have stored plant position in data field
                    data = cell[5] if len(cell) > 5 else ""
                    if data and ',' in data:
                        try:
                            px, py = map(float, data.split(','))
                            # Plant obstacle - treat as ~0.8m footprint (radius 0.4m) for safer clearance
                            self.planner.add_obstacle(px, py, radius=0.4)
                        except:
                            # Fallback to cell center
                            cx = config.GRID_CONFIG['min_x'] + (cell[0] + 0.5) * config.CELL_W
                            cy = config.GRID_CONFIG['min_y'] + (cell[1] + 0.5) * config.CELL_H
                            self.planner.add_obstacle(cx, cy, radius=0.4)
                    else:
                        cx = config.GRID_CONFIG['min_x'] + (cell[0] + 0.5) * config.CELL_W
                        cy = config.GRID_CONFIG['min_y'] + (cell[1] + 0.5) * config.CELL_H
                        self.planner.add_obstacle(cx, cy, radius=0.4)
        self.state = "NEXT_GLOBAL_POINT"; self.log_msg.emit(f"Job: {job_type} | Waypoints: {len(points)}"); self.mutex.unlock()

    def stop_robot(self): self.pub_vel.publish(Twist())

    def control_loop(self):
        self.mutex.lock()
        dt = time.time() - self.last_time; self.last_time = time.time()
        self.update_path_viz.emit(self.current_goal or {}, self.local_path if self.state=="FOLLOW_PATH" else self.global_waypoints)
        self.update_seeder.emit(self.seeder_state, self.seeder_height)
        cmd = Twist()

        if self.state == "MANUAL":
            cmd.linear.x = float(self.manual_vx); cmd.angular.z = float(self.manual_vw); self.pub_vel.publish(cmd)
            if self.manual_seeder_vel != 0: self.seeder_height = max(-0.15, min(0.20, self.seeder_height + self.manual_seeder_vel * dt * 0.1))
            self.mutex.unlock(); return

        # AVOIDING state: Smart obstacle avoidance with directional rotation
        if self.state == "AVOIDING":
            action, angular_vel = self.obstacle_avoider.update(
                self.ryaw,
                self.us_ranges['us3'],  # Front-left
                self.us_ranges['us2'],  # Front-right
                self.us_ranges['us4'],  # Left
                self.us_ranges['us1'],  # Right
                obstacle_dist=1.5
            )
            
            if action == 'ROTATE':
                cmd.linear.x = 0.0
                cmd.angular.z = float(angular_vel)
                self.pub_vel.publish(cmd)
            elif action == 'REVERSE':
                self.state = "REVERSING"
                self.reversing = True
                self.reverse_start_time = time.time()
                self.prev_state = "FOLLOW_PATH"
                self.log_msg.emit("Rotating failed - reversing")
            elif action == 'REPLAN':
                self.avoiding_obstacle = False
                self.obstacle_avoider.reset()
                # Replan path around obstacle
                if self.current_goal:
                    new_path = self.planner.plan(self.rx, self.ry, self.current_goal['x'], self.current_goal['y'], robot_yaw=self.ryaw)
                    if new_path:
                        self.local_path = new_path
                        self.state = "ROTATING"
                        self.log_msg.emit("Path replanned after avoidance")
                    else:
                        self.state = "NEXT_GLOBAL_POINT"
                else:
                    self.state = "IDLE"
            else:  # CONTINUE - shouldn't happen in AVOIDING state
                self.avoiding_obstacle = False
                self.state = "FOLLOW_PATH"
            self.mutex.unlock(); return

        if self.state == "WAIT_REPLAN":
            self.stop_robot()
            if not self.current_goal:
                self.state = "IDLE"
                self.mutex.unlock(); return
            now = time.time()
            if now - self.last_replan_attempt > 1.0:
                self.last_replan_attempt = now
                new_path = self.planner.plan(self.rx, self.ry, self.current_goal['x'], self.current_goal['y'], robot_yaw=self.ryaw)
                if new_path:
                    self.local_path = new_path
                    self.awaiting_replan = False
                    self.state = "ROTATING"
                    self.log_msg.emit("Path updated after obstacle")
            self.mutex.unlock(); return

        # REVERSING state: back up until front sensors (us2, us3) read >= 1.0m
        if self.state == "REVERSING":
            front_min = min(self.us_ranges['us2'], self.us_ranges['us3'])
            if front_min >= 1.0 or (time.time() - self.reverse_start_time) > 3.0:
                # Done reversing - replan path
                self.reversing = False
                self.avoiding_obstacle = False
                self.obstacle_avoider.reset()
                self.stop_robot()
                self.log_msg.emit(f"REVERSE COMPLETE: front clear at {front_min:.2f}m")
                if hasattr(self, 'prev_state') and self.prev_state == "FOLLOW_PATH" and self.current_goal:
                    new_path = self.planner.plan(self.rx, self.ry, self.current_goal['x'], self.current_goal['y'], robot_yaw=self.ryaw)
                    if new_path:
                        self.local_path = new_path
                        self.state = "ROTATING"
                    else:
                        self.state = "NEXT_GLOBAL_POINT"
                else:
                    self.state = "IDLE"
            else:
                # Keep reversing
                cmd.linear.x = -0.3  # Reverse speed
                cmd.angular.z = 0.0
                self.pub_vel.publish(cmd)
            self.mutex.unlock(); return

        if self.state == "NEXT_GLOBAL_POINT":
            if self.global_waypoints:
                self.current_goal = self.global_waypoints.pop(0)
                # Plan path from robot front point (pass yaw for front offset calculation)
                self.log_msg.emit(f"Planning: Robot({self.rx:.2f},{self.ry:.2f}) -> Goal({self.current_goal['x']:.2f},{self.current_goal['y']:.2f})")
                path = self.planner.plan(self.rx, self.ry, self.current_goal['x'], self.current_goal['y'], robot_yaw=self.ryaw)
                self.log_msg.emit(f"Path result: {len(path)} points, Obstacles in grid: {self.planner.obstacle_grid.sum()}")
                if path: 
                    self.local_path = path
                    self.state = "ROTATING"  # Rotate first before following path
                else: self.log_msg.emit("Point Unreachable"); self.state = "NEXT_GLOBAL_POINT"
            else: 
                self.state = "IDLE"; self.stop_robot()
                # Show soil scan summary if we have results
                if self.soil_scan_results and self.current_job_type == "SOIL":
                    self._show_soil_scan_summary()
                if self.current_job_type == "PLANTING":
                    self.show_popup.emit("Planting complete", f"Finished planting {self.plant_targets_count} square(s)")
                self.job_finished.emit(str(self.current_job_type))

        elif self.state == "ROTATING":
            # Rotate in place to face the first waypoint before moving
            if not self.local_path:
                self.state = "NEXT_GLOBAL_POINT"
                self.mutex.unlock(); return
            
            target = self.local_path[0]
            dx = target['x'] - self.rx
            dy = target['y'] - self.ry
            target_yaw = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_yaw - self.ryaw)
            
            if abs(angle_diff) < 0.15:  # ~8.5 degrees - close enough, start moving
                self.state = "FOLLOW_PATH"
                self.prev_angular_error = 0.0
            else:
                # Rotate in place
                cmd.linear.x = 0.0
                cmd.angular.z = float(max(min(self.kp_angular * angle_diff, self.max_angular), -self.max_angular))
                self.pub_vel.publish(cmd)
            self.mutex.unlock(); return
        elif self.state == "FOLLOW_PATH":
            # --- STEP 1: Find the Closest Point (To measure Error) ---
            # We need to know how far we are from the path generally to tune the lookahead.
            closest_dist = float('inf')
            closest_idx = 0
            
            # Search part of the path to find where we are
            # Optimization: check the first 20 points or full length
            search_len = min(len(self.local_path), 20) 
            for i in range(search_len):
                pt = self.local_path[i]
                d = math.hypot(pt['x'] - self.rx, pt['y'] - self.ry)
                if d < closest_dist:
                    closest_dist = d
                    closest_idx = i

            # --- STEP 2: Calculate Adaptive Lookahead ---
            # closest_dist is our "Cross Track Error" proxy.
            # If we are far from the path, look further ahead to smooth the re-entry.
            min_lookahead = 0.4   # Precision mode
            max_lookahead = 1.2   # Recovery mode
            
            # The Formula: Base + (Error * Gain)
            lookahead_dist = min_lookahead + (closest_dist * 0.6)
            lookahead_dist = max(min(lookahead_dist, max_lookahead), min_lookahead)

            # --- STEP 3: Find the Target Point (Using the calculated Lookahead) ---
            target_point = self.local_path[-1] # Default to end
            
            # Start searching forward from the closest point index
            for i in range(closest_idx, len(self.local_path)):
                pt = self.local_path[i]
                d = math.hypot(pt['x'] - self.rx, pt['y'] - self.ry)
                
                # Found a point further than our lookahead distance? That's the target.
                if d >= lookahead_dist:
                    target_point = pt
                    break

            # --- STEP 4: Calculate Local X and Y (Now valid) ---
            dx = target_point['x'] - self.rx
            dy = target_point['y'] - self.ry

            local_x = dx * math.cos(self.ryaw) + dy * math.sin(self.ryaw)
            local_y = -dx * math.sin(self.ryaw) + dy * math.cos(self.ryaw)

            # --- STEP 5: Arrival Check ---
            dist_to_goal = math.hypot(self.current_goal['x'] - self.rx, self.current_goal['y'] - self.ry)
            tol = 0.15 if self.current_goal.get('type') in ['plant', 'plant_scan'] else 0.25
            
            if dist_to_goal < tol:
                # ... (Keep your existing arrival logic here) ...
                if self.current_goal.get('type') in ['soil_scan', 'plant_scan', 'plant']:
                    self.point_reached.emit(self.current_goal['x'], self.current_goal['y'], self.current_goal['type'])
                    self.state = "SEEDER_ACTION"; self.seeder_state = "IDLE"
                else:
                    self.state = "NEXT_GLOBAL_POINT"
                self.mutex.unlock(); return

            # --- STEP 6: Pure Pursuit Curvature Control ---
            curvature = (2.0 * local_y) / (lookahead_dist ** 2)

            # --- STEP 7: Improved Speed Control (Non-Stopping Turns) ---
            # (Use the non-stop logic I gave you in the previous answer here)
            
            # Base speed
            target_vel = self.max_linear
            
            # Slow down based on curvature (sharp turns)
            if abs(curvature) > 1.0: target_vel *= 0.4
            elif abs(curvature) > 0.5: target_vel *= 0.7
            
            # Apply to command
            cmd.linear.x = float(target_vel)
            cmd.angular.z = float(target_vel * curvature)
            
            # Clamp angular
            cmd.angular.z = max(min(cmd.angular.z, self.max_angular), -self.max_angular)
            
            self.pub_vel.publish(cmd)

            # Path Cleanup: Remove points behind us (0 to closest_idx)
            # This keeps the loop efficient
            if closest_idx > 0:
                del self.local_path[:closest_idx]

        elif self.state == "SEEDER_ACTION":
            self.stop_robot(); move_speed = 0.2 * dt 

            target_depth = -0.15
            if self.current_goal.get('type') in ['plant', 'plant_scan']:
                target_depth = -self.selected_seed_depth_m
            target_depth = max(-0.20, min(-0.02, target_depth))

            if self.seeder_state == "IDLE": self.seeder_state = "LOWERING"
            elif self.seeder_state == "LOWERING":
                self.seeder_height -= move_speed
                if self.seeder_height <= 0.0: self.seeder_state = "WORKING"
            elif self.seeder_state == "WORKING":
                self.seeder_height -= move_speed
                if self.seeder_height <= target_depth:
                    self.seeder_height = target_depth
                    self.seeder_state = "WAITING"; self.seeder_timer = time.time()
            elif self.seeder_state == "WAITING":
                if time.time() - self.seeder_timer > 1.5: self.seeder_state = "RAISING"
            elif self.seeder_state == "RAISING":
                self.seeder_height += move_speed
                if self.seeder_height >= 0.20:
                    self.seeder_height = 0.20; self.seeder_state = "IDLE"
                    if self.current_goal.get('type') == 'soil_scan':
                        # Generate 7 soil readings
                        n_val = random.randint(10, 80)           # Nitrogen (ppm)
                        p_val = random.randint(5, 50)            # Phosphorus (ppm)
                        k_val = random.randint(50, 200)          # Potassium (ppm)
                        ph_val = round(random.uniform(5.5, 8.0), 1)  # pH
                        moisture_val = round(random.uniform(15, 60), 1)  # Moisture (%)
                        temp_val = round(random.uniform(15, 35), 1)  # Temperature (°C)
                        ec_val = round(random.uniform(0.5, 4.0), 2)  # EC (mS/cm)
                        
                        # Calculate grid cell
                        c = int((self.current_goal['x'] - config.GRID_CONFIG['min_x']) / config.CELL_W)
                        r = int((config.GRID_CONFIG['max_y'] - self.current_goal['y']) / config.CELL_H)
                        
                        # Save to database
                        if self.db_ref:
                            self.db_ref.add_soil_scan(
                                grid_x=c, grid_y=r,
                                world_x=self.current_goal['x'], world_y=self.current_goal['y'],
                                nitrogen=n_val, phosphorus=p_val, potassium=k_val,
                                ph=ph_val, moisture=moisture_val, temperature=temp_val, ec=ec_val
                            )
                            self.log_msg.emit(f"Soil scan saved at Grid[{c},{r}]")
                        
                        # Collect scan result for summary (don't show popup here)
                        self.soil_scan_results.append({
                            'grid': (c, r),
                            'n': n_val, 'p': p_val, 'k': k_val,
                            'ph': ph_val, 'moisture': moisture_val,
                            'temp': temp_val, 'ec': ec_val
                        })
                    elif self.current_goal.get('type') in ['plant', 'plant_scan']:
                        # Mark planted in DB
                        c = int((self.current_goal['x'] - config.GRID_CONFIG['min_x']) / config.CELL_W)
                        r = int((config.GRID_CONFIG['max_y'] - self.current_goal['y']) / config.CELL_H)
                        if self.db_ref:
                            batch_id = datetime.now().strftime("PLANT_%Y%m%d_%H%M")
                            self.db_ref.update_cell(c, r, 'planted', batch_id, data=f"{self.current_goal['x']:.3f},{self.current_goal['y']:.3f}", seed_type=self.selected_seed or '')
                        self.log_msg.emit(f"Planted seed {self.selected_seed or 'DEFAULT'} at Grid[{c},{r}]")
                    self.state = "NEXT_GLOBAL_POINT"
        self.mutex.unlock()
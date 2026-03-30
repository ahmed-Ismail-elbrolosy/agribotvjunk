import sys
import os
import math
import time
import random
import csv
import numpy as np
from datetime import datetime

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image, Range 

# PyQt5 Imports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QLabel, QScrollArea, QGroupBox, QProgressBar, QFormLayout, 
    QSpinBox, QPushButton, QGridLayout, QTextEdit, QFileDialog, QMessageBox,
    QSizePolicy
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer, QRect, QPoint
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QPainterPath

# Local Imports (Ensure these files exist in the same folder)
from database import AgriBotDB
from astar import AStarPlanner 

# Try CvBridge
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False
    print("Warning: cv_bridge not found. Camera feed will be disabled.")

# ============================================================================
# 1. CONFIGURATION (GLOBAL)
# ============================================================================

# World Bounds (Meters)
WORLD_MIN_X = -2.0; WORLD_MAX_X = 12.0   
WORLD_MIN_Y = -7.0; WORLD_MAX_Y = 7.0    

# Robot Dimensions
ROBOT_LENGTH = 1.2 
ROBOT_WIDTH = 0.7   

# Sensor Offsets (Relative to Robot Center)
SENSOR_OFFSETS = {
    'us1': {'x': 0.138, 'y': -0.422, 'yaw': -0.6283}, 
    'us2': {'x': 0.45,  'y': -0.282, 'yaw': 0.0},
    'us3': {'x': 0.45,  'y': 0.233,  'yaw': -0.3725}, 
    'us4': {'x': 0.138, 'y': 0.373,  'yaw': 0.6283}
}
US_FOV = math.radians(60)
US_MAX_RANGE = 4.0

# Field Grid Configuration
GRID_CONFIG = {
    'min_x': 2.0,   'min_y': -3.0, 
    'max_x': 4.0,   'max_y': 1.0,  
    'rows': 3,      'cols': 3
}
CELL_W = (GRID_CONFIG['max_x'] - GRID_CONFIG['min_x']) / GRID_CONFIG['cols']
CELL_H = (GRID_CONFIG['max_y'] - GRID_CONFIG['min_y']) / GRID_CONFIG['rows']

# Background Image
BG_PATH = "sandbackground.jpg" 

# ============================================================================
# 2. ROS CONTROLLER
# ============================================================================
class RosController(QThread):
    # Signals to GUI
    update_odom = pyqtSignal(float, float, float)
    update_cam = pyqtSignal(QImage)
    update_path_viz = pyqtSignal(dict, list) 
    update_seeder = pyqtSignal(str, float) 
    update_sensors = pyqtSignal(dict) 
    point_reached = pyqtSignal(float, float, str) 
    job_finished = pyqtSignal(str) 
    log_msg = pyqtSignal(str)
    show_popup = pyqtSignal(str, str) 

    def __init__(self):
        super().__init__()
        self.node = None
        
        # Navigation State
        self.global_waypoints = []
        self.local_path = []
        self.current_goal = None    
        self.state = "IDLE" 
        self.current_job_type = None 
        
        # A* Planner
        self.planner = AStarPlanner(WORLD_MIN_X, WORLD_MAX_X, WORLD_MIN_Y, WORLD_MAX_Y, resolution=0.2)
        
        # Sensors
        self.us_ranges = {'us1': 4.0, 'us2': 4.0, 'us3': 4.0, 'us4': 4.0}
        self.db_ref = None 

        # Seeder State
        self.seeder_state = "IDLE"
        self.seeder_height = 0.20 
        self.seeder_timer = 0
        
        # Manual Control
        self.manual_vx = 0.0; self.manual_vw = 0.0; self.manual_seeder_vel = 0.0 
        
        # PID Constants
        self.kp_linear = 2.0;   self.kd_linear = 1.5   
        self.kp_angular = 3.0;  self.kd_angular = 0.5  
        self.max_linear = 2.0;  self.max_angular = 1.0
        self.prev_angular_error = 0.0
        self.last_time = 0
        self.rx = 0.0; self.ry = 0.0; self.ryaw = 0.0

        # CV Config
        self.IMG_W = 640; self.IMG_H = 480
        self.FOV_H = 1.047 
        self.FOCAL_LENGTH = (self.IMG_W / 2) / math.tan(self.FOV_H / 2) 
        self.REAL_PLANT_WIDTH = 0.20 
        self.CAM_OFF_X = 0.45; self.CAM_OFF_Y = -0.0334

    def run(self):
        if not rclpy.ok(): rclpy.init()
        self.node = Node('agribot_dashboard_controller')
        self.pub_vel = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        self.node.create_subscription(TFMessage, '/world/SinaiAgri/dynamic_pose/info', self.tf_cb, 10)
        self.node.create_subscription(Image, '/camera/image_raw', self.cam_cb, 10)
        
        for sensor in ['us1', 'us2', 'us3', 'us4']:
            self.node.create_subscription(Range, f'/ultra/{sensor}', lambda msg, s=sensor: self.us_cb(msg, s), 10)

        self.last_time = time.time()
        self.timer = self.node.create_timer(0.05, self.control_loop) 
        try: rclpy.spin(self.node)
        except Exception: pass
        finally:
            self.stop_robot()
            self.node.destroy_node()
            rclpy.shutdown()

    def us_cb(self, msg, sensor_id):
        r = min(msg.range, US_MAX_RANGE)
        self.us_ranges[sensor_id] = r
        self.update_sensors.emit(self.us_ranges)
        # Obstacle Trigger
        if sensor_id in ['us1', 'us2'] and r < 2.0:
            self.register_dynamic_obstacle(sensor_id, r)

    def register_dynamic_obstacle(self, sensor, dist):
        off = SENSOR_OFFSETS[sensor]
        global_s_yaw = self.ryaw + off['yaw']
        obs_x = self.rx + (dist * math.cos(global_s_yaw))
        obs_y = self.ry + (dist * math.sin(global_s_yaw))
        
        self.planner.add_obstacle(obs_x, obs_y, radius=0.4)
        
        # SMART VALIDATION: Only replan if the CURRENT path is actually blocked
        if self.state == "FOLLOW_PATH" and self.local_path:
            is_blocked = False
            # Check the next 15 nodes (about 1.5 - 3 meters)
            check_nodes = self.local_path[:15]
            for pt in check_nodes:
                d = math.sqrt((pt['x'] - obs_x)**2 + (pt['y'] - obs_y)**2)
                if d < 0.7: # Obstacle radius + Robot radius margin
                    is_blocked = True
                    break
            
            if is_blocked:
                # self.log_msg.emit("Path blocked! Replanning...")
                new_path = self.planner.plan(self.rx, self.ry, self.current_goal['x'], self.current_goal['y'])
                if new_path: self.local_path = new_path

    def tf_cb(self, msg):
        for transform in msg.transforms:
            if 'AgriBot' in transform.child_frame_id or transform.child_frame_id == 'AgriBot':
                self.rx = transform.transform.translation.x
                self.ry = transform.transform.translation.y
                q = transform.transform.rotation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y); cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                self.ryaw = math.atan2(siny_cosp, cosy_cosp)
                self.update_odom.emit(self.rx, self.ry, self.ryaw)
                break

    def cam_cb(self, msg):
        if not CV_BRIDGE_AVAILABLE: return
        try:
            bridge = CvBridge()
            cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            processed_img, detected = self.process_cv(cv_img)
            self.update_map_from_cv(detected)
            h, w, ch = processed_img.shape
            # COPY is essential to prevent flickering/garbage collection
            qt_img = QImage(processed_img.data, w, h, ch * w, QImage.Format_RGB888).rgbSwapped().copy()
            self.update_cam.emit(qt_img)
        except Exception: pass

    def process_cv(self, img):
        import cv2
        detections = []
        blurred = cv2.GaussianBlur(img, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([30, 40, 40]), np.array([90, 255, 255]))
        mask = cv2.erode(mask, None, iterations=2); mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            if cv2.contourArea(c) < 500: continue
            (x, y, w, h) = cv2.boundingRect(c)
            d = (self.FOCAL_LENGTH * self.REAL_PLANT_WIDTH) / w
            if d > 3.0: continue 
            cx = x + (w // 2); offset_px = (self.IMG_W / 2) - cx 
            alpha = math.atan2(offset_px, self.FOCAL_LENGTH)
            total_yaw = self.ryaw + alpha
            cam_wx = self.rx + (self.CAM_OFF_X * math.cos(self.ryaw) - self.CAM_OFF_Y * math.sin(self.ryaw))
            cam_wy = self.ry + (self.CAM_OFF_X * math.sin(self.ryaw) + self.CAM_OFF_Y * math.cos(self.ryaw))
            plant_wx = cam_wx + (d * math.cos(total_yaw))
            plant_wy = cam_wy + (d * math.sin(total_yaw))
            detections.append((plant_wx, plant_wy))
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(img, f"{d:.1f}m", (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        return img, detections

    def update_map_from_cv(self, detections):
        if not self.db_ref: return
        for (wx, wy) in detections:
            c = int((wx - GRID_CONFIG['min_x']) / CELL_W)
            # Row 0 is at max_y (top), rows increase downward - matching map_widget
            r = int((GRID_CONFIG['max_y'] - wy) / CELL_H)
            if 0 <= c < GRID_CONFIG['cols'] and 0 <= r < GRID_CONFIG['rows']:
                self.db_ref.update_cell(c, r, 'planted', 'CV_DETECT')
                cx = GRID_CONFIG['min_x'] + (c + 0.5) * CELL_W
                # Calculate cy from max_y since row 0 starts at top
                cy = GRID_CONFIG['max_y'] - (r + 0.5) * CELL_H
                self.planner.add_obstacle(cx, cy, radius=0.1)

    def set_manual_vel(self, vx, vw):
        if self.state != "MANUAL": self.state = "MANUAL"; self.global_waypoints = []; self.local_path = []; self.log_msg.emit("Manual Control")
        self.manual_vx = vx; self.manual_vw = vw

    def set_manual_seeder(self, vel):
        if self.state != "MANUAL": self.state = "MANUAL"; self.log_msg.emit("Manual Control")
        self.manual_seeder_vel = vel

    def set_path(self, points, job_type="MOVE"):
        self.state = "IDLE"; self.current_goal = None; self.local_path = []
        self.global_waypoints = points; self.current_job_type = job_type
        
        self.planner.clear_obstacles()
        if self.db_ref:
            cells = self.db_ref.get_all_cells()
            for cell in cells:
                if cell[2] == 'planted':
                    cx = GRID_CONFIG['min_x'] + (cell[0] + 0.5) * CELL_W
                    # Row 0 is at max_y, rows increase downward
                    cy = GRID_CONFIG['max_y'] - (cell[1] + 0.5) * CELL_H
                    self.planner.add_obstacle(cx, cy, radius=0.1)

        self.state = "NEXT_GLOBAL_POINT"
        self.log_msg.emit(f"Job: {job_type} | Points: {len(points)}")

    def control_loop(self):
        dt = time.time() - self.last_time; self.last_time = time.time()
        
        viz_path = self.local_path if self.state == "FOLLOW_PATH" else self.global_waypoints
        self.update_path_viz.emit(self.current_goal or {}, viz_path)
        self.update_seeder.emit(self.seeder_state, self.seeder_height)

        cmd = Twist()

        if self.state == "MANUAL":
            cmd.linear.x = float(self.manual_vx); cmd.angular.z = float(self.manual_vw)
            self.pub_vel.publish(cmd)
            if self.manual_seeder_vel != 0:
                self.seeder_height += self.manual_seeder_vel * dt * 0.1
                self.seeder_height = max(-0.15, min(0.20, self.seeder_height))
            return

        if self.state == "NEXT_GLOBAL_POINT":
            if self.global_waypoints:
                self.current_goal = self.global_waypoints.pop(0)
                path = self.planner.plan(self.rx, self.ry, self.current_goal['x'], self.current_goal['y'])
                if path: self.local_path = path; self.state = "FOLLOW_PATH"
                else: self.log_msg.emit("Path Blocked"); self.state = "NEXT_GLOBAL_POINT" 
            else:
                self.state = "IDLE"; self.pub_vel.publish(Twist())
                self.job_finished.emit(str(self.current_job_type))

        elif self.state == "FOLLOW_PATH":
            if not self.local_path:
                dist_g = math.sqrt((self.current_goal['x']-self.rx)**2 + (self.current_goal['y']-self.ry)**2)
                if dist_g < 0.2:
                    if self.current_goal.get('type') in ['soil_scan', 'plant_scan']:
                         self.point_reached.emit(self.current_goal['x'], self.current_goal['y'], self.current_goal['type'])
                         self.state = "SEEDER_ACTION"; self.seeder_state = "IDLE"
                    else: self.state = "NEXT_GLOBAL_POINT"
                else:
                    new_path = self.planner.plan(self.rx, self.ry, self.current_goal['x'], self.current_goal['y'])
                    if new_path: self.local_path = new_path
                    else: self.state = "NEXT_GLOBAL_POINT"
                return

            target = self.local_path[0]
            dx = target['x'] - self.rx; dy = target['y'] - self.ry
            dist = math.sqrt(dx**2 + dy**2)
            if dist < 0.25: self.local_path.pop(0); return

            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self.ryaw
            while angle_diff > math.pi: angle_diff -= 2*math.pi
            while angle_diff < -math.pi: angle_diff += 2*math.pi
            
            ang_d = (angle_diff - self.prev_angular_error) / dt; self.prev_angular_error = angle_diff
            cmd_w = (self.kp_angular * angle_diff) + (self.kd_angular * ang_d)
            cmd_w = max(min(cmd_w, self.max_angular), -self.max_angular)
            cmd_v = self.max_linear if abs(angle_diff) < 0.5 else 0.1
            cmd.linear.x = float(cmd_v); cmd.angular.z = float(cmd_w)
            self.pub_vel.publish(cmd)

        elif self.state == "SEEDER_ACTION":
            self.pub_vel.publish(Twist()) 
            move_speed = 0.2 * dt 
            
            if self.seeder_state == "IDLE": self.seeder_state = "LOWERING"; self.log_msg.emit("Task: Lowering Tool...")
            elif self.seeder_state == "LOWERING":
                self.seeder_height -= move_speed
                if self.seeder_height <= 0.0: self.seeder_state = "WORKING"; self.log_msg.emit("Task: Working...")
            elif self.seeder_state == "WORKING":
                self.seeder_height -= move_speed
                if self.seeder_height <= -0.15: self.seeder_height = -0.15; self.seeder_state = "WAITING"; self.seeder_timer = time.time()
            elif self.seeder_state == "WAITING":
                if time.time() - self.seeder_timer > 1.5: self.seeder_state = "RAISING"
            elif self.seeder_state == "RAISING":
                self.seeder_height += move_speed
                if self.seeder_height >= 0.20:
                    self.seeder_height = 0.20
                    self.seeder_state = "IDLE"
                    
                    # --- SHOW POPUP ---
                    if self.current_goal.get('type') == 'soil_scan':
                        n = random.randint(10,50); p = random.randint(10,40); k = random.randint(10,40)
                        msg = f"Soil Analysis at ({self.current_goal['x']:.1f}, {self.current_goal['y']:.1f}):\n\nNitrogen: {n} ppm\nPhosphorus: {p} ppm\nPotassium: {k} ppm\npH: {random.uniform(6.0, 7.5):.1f}"
                        self.show_popup.emit("Analysis Complete", msg)
                    
                    self.state = "NEXT_GLOBAL_POINT"

    def stop_robot(self): self.pub_vel.publish(Twist())

# ============================================================================
# 3. MAP WIDGET
# ============================================================================
class MapWidget(QWidget):
    mouse_moved_signal = pyqtSignal(float, float) 

    def __init__(self, db, controller):
        super().__init__()
        self.db = db; self.ctrl = controller
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self.rx = 0; self.ry = 0; self.ryaw = 0
        self.us_data = {}; self.current_goal = None; self.future_path = []
        self.mode = "VIEW"; self.selection_start = None; self.selection_end = None
        
        # FIX: Check if BG_PATH is defined and file exists, else use None
        if 'BG_PATH' in globals() and os.path.exists(BG_PATH):
            self.bg_pixmap = QPixmap(BG_PATH)
        else:
            self.bg_pixmap = None

        self.scale = 1.0; self.off_x = 0; self.off_y = 0
        self.blink_pos = None; self.blink_timer = QTimer(); self.blink_timer.timeout.connect(self.stop_blink)

    def start_blink(self, x, y): self.blink_pos = (x, y); self.blink_timer.start(1000); self.update()
    def stop_blink(self): self.blink_pos = None; self.blink_timer.stop(); self.update()
    def update_sensors(self, data): self.us_data = data; self.update()
    def update_path(self, goal, path): self.current_goal = goal; self.future_path = path; self.update()
    def update_robot(self, x, y, yaw): self.rx = x; self.ry = y; self.ryaw = yaw; self.update()

    def update_transform_params(self):
        w_px = self.width(); h_px = self.height()
        world_w = WORLD_MAX_X - WORLD_MIN_X; world_h = WORLD_MAX_Y - WORLD_MIN_Y
        self.scale = min(w_px / world_w, h_px / world_h) 
        self.off_x = (w_px - (world_w * self.scale)) / 2; self.off_y = (h_px - (world_h * self.scale)) / 2

    def world_to_screen(self, wx, wy):
        if self.scale == 0: return 0,0 
        sx = self.off_x + ((wx - WORLD_MIN_X) * self.scale)
        sy = self.off_y + ((WORLD_MAX_Y - wy) * self.scale)
        return int(sx), int(sy)

    def screen_to_world(self, sx, sy):
        if self.scale == 0: return 0,0
        wx = WORLD_MIN_X + ((sx - self.off_x) / self.scale)
        wy = WORLD_MAX_Y - ((sy - self.off_y) / self.scale)
        return wx, wy

    def paintEvent(self, event):
        self.update_transform_params()
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        map_rect = QRect(int(self.off_x), int(self.off_y), int((WORLD_MAX_X-WORLD_MIN_X)*self.scale), int((WORLD_MAX_Y-WORLD_MIN_Y)*self.scale))
        if self.bg_pixmap: p.drawPixmap(map_rect, self.bg_pixmap)
        else: p.fillRect(map_rect, QColor(238, 214, 175))
        p.setPen(QPen(Qt.black, 3)); p.setBrush(Qt.NoBrush); p.drawRect(map_rect)

        self.draw_grid_lines(p)
        cells = self.db.get_all_cells(); known_map = {(c[0], c[1]): c for c in cells}
        
        for r in range(GRID_CONFIG['rows']):
            for c in range(GRID_CONFIG['cols']):
                wx_min = GRID_CONFIG['min_x'] + (c * CELL_W); wx_max = wx_min + CELL_W
                wy_min = GRID_CONFIG['min_y'] + (r * CELL_H); wy_max = wy_min + CELL_H
                s_tl_x, s_tl_y = self.world_to_screen(wx_min, wy_max)
                s_br_x, s_br_y = self.world_to_screen(wx_max, wy_min)
                rect = QRect(QPoint(s_tl_x, s_tl_y), QPoint(s_br_x, s_br_y)).normalized()
                
                color = QColor(255, 255, 224, 40) 
                is_planted = False
                if (c, r) in known_map:
                    status = known_map[(c,r)][2]
                    if status == 'soil_analyzed': color = QColor(0, 0, 255, 100) 
                    elif status == 'planted': color = QColor(0, 255, 0, 100); is_planted = True
                    elif status == 'plant_checked': color = QColor(255, 215, 0, 100)
                p.fillRect(rect, color); p.setPen(QPen(QColor(0,0,0,50), 1)); p.drawRect(rect)
                if is_planted:
                    cx, cy = self.world_to_screen(wx_min + CELL_W/2, wy_min + CELL_H/2)
                    sz = int(0.3 * self.scale); p.setBrush(Qt.darkGreen); p.setPen(Qt.NoPen); p.drawRect(cx - sz//2, cy - sz//2, sz, sz)

        pts = [QPoint(*self.world_to_screen(self.rx, self.ry))]
        for pt in self.future_path: pts.append(QPoint(*self.world_to_screen(pt['x'], pt['y'])))
        if len(pts) > 1:
            path_obj = QPainterPath(); path_obj.moveTo(pts[0])
            for pt in pts[1:]: path_obj.lineTo(pt)
            p.setPen(QPen(Qt.green, 2)); p.setBrush(Qt.NoBrush); p.drawPath(path_obj)

        if self.selection_start and self.selection_end:
            rect = QRect(self.selection_start, self.selection_end).normalized()
            p.setPen(QPen(Qt.blue, 2, Qt.DashLine)); p.setBrush(QColor(0,0,255,50)); p.drawRect(rect)

        if self.blink_pos:
             bx, by = self.world_to_screen(self.blink_pos[0], self.blink_pos[1])
             p.setPen(QPen(Qt.red, 3)); p.setBrush(QColor(255,0,0,100)); p.drawEllipse(QPoint(bx,by), 15, 15)

        self.draw_robot_and_sensors(p)

    def draw_robot_and_sensors(self, p):
        sx, sy = self.world_to_screen(self.rx, self.ry)
        p.save(); p.translate(sx, sy); p.rotate(-math.degrees(self.ryaw)) 
        p.setBrush(Qt.blue); p.setPen(Qt.black)
        w = int(ROBOT_WIDTH * self.scale); l = int(ROBOT_LENGTH * self.scale)
        p.drawRect(-l//2, -w//2, l, w)
        p.setBrush(Qt.cyan); p.drawEllipse(QPoint(int(-0.4*self.scale), 0), 5, 5) 

        for name, conf in SENSOR_OFFSETS.items():
            r = self.us_data.get(name, 4.0)
            if r < 1.0: col = QColor(255, 0, 0, 150)
            elif r < 2.0: col = QColor(255, 165, 0, 100)
            else: col = QColor(0, 255, 255, 40)
            px = int(conf['x']*self.scale); py = int(conf['y']*self.scale) 
            p.save(); p.translate(px, -py); p.rotate(-math.degrees(conf['yaw']))
            r_px = int(r * self.scale); span = int(math.degrees(US_FOV) * 16)
            p.setBrush(col); p.setPen(Qt.NoPen); p.drawPie(-r_px, -r_px, r_px*2, r_px*2, -span//2, span)
            p.restore()
        p.restore()

    def draw_grid_lines(self, p):
        p.setPen(QPen(QColor(50, 50, 50, 100), 1)) 
        for y in range(int(WORLD_MIN_Y), int(WORLD_MAX_Y) + 1):
            sx1, sy1 = self.world_to_screen(WORLD_MIN_X, y); sx2, sy2 = self.world_to_screen(WORLD_MAX_X, y); p.drawLine(sx1, sy1, sx2, sy2)
        for x in range(int(WORLD_MIN_X), int(WORLD_MAX_X) + 1):
            sx1, sy1 = self.world_to_screen(x, WORLD_MIN_Y); sx2, sy2 = self.world_to_screen(x, WORLD_MAX_Y); p.drawLine(sx1, sy1, sx2, sy2)

    def mousePressEvent(self, event):
        wx, wy = self.screen_to_world(event.x(), event.y())
        if self.mode == "GOTO":
            self.ctrl.set_path([{'x': wx, 'y': wy, 'type': 'move'}], "MOVE")
            self.mode = "VIEW"; self.setCursor(Qt.ArrowCursor); self.update()
        elif self.mode != "VIEW":
            self.selection_start = event.pos(); self.selection_end = event.pos()

    def mouseReleaseEvent(self, event):
        if self.selection_start: self.process_selection(); self.selection_start = None; self.selection_end = None; self.update()
    
    def mouseMoveEvent(self, event):
        wx, wy = self.screen_to_world(event.x(), event.y())
        self.mouse_moved_signal.emit(wx, wy)
        if self.selection_start: self.selection_end = event.pos(); self.update()

    def process_selection(self):
        wx1, wy1 = self.screen_to_world(self.selection_start.x(), self.selection_start.y())
        wx2, wy2 = self.screen_to_world(self.selection_end.x(), self.selection_end.y())
        min_wx, max_wx = min(wx1, wx2), max(wx1, wx2); min_wy, max_wy = min(wy1, wy2), max(wy1, wy2)
        path = []; job = "MOVE"
        
        if self.mode == "PLANT":
            job = "PLANTING"
            for r in range(GRID_CONFIG['rows']):
                for c in range(GRID_CONFIG['cols']):
                    cx = GRID_CONFIG['min_x'] + (c + 0.5) * CELL_W; cy = GRID_CONFIG['min_y'] + (r + 0.5) * CELL_H
                    if min_wx <= cx <= max_wx and min_wy <= cy <= max_wy:
                        # Skip cells already planted/detected in DB
                        state = self.db.get_cell_status(r, c) if hasattr(self.db, 'get_cell_status') else None
                        if state in ['planted', 'detected', 'obstacle']:
                            continue
                        path.append({'x': cx, 'y': cy, 'type': 'plant', 'grid': (c, r)})
        elif self.mode == "SOIL_SCAN":
            job = "SOIL"
            for _ in range(self.parent().parent().parent().n_points_spin.value()):
                 path.append({'x': random.uniform(min_wx, max_wx), 'y': random.uniform(min_wy, max_wy), 'type': 'soil_scan'})

        if path: self.ctrl.set_path(path, job)
        self.mode = "VIEW"; self.setCursor(Qt.ArrowCursor)

# ============================================================================
# 4. MAIN WINDOW
# ============================================================================
class LegendWidget(QWidget):
    def __init__(self):
        super().__init__()
        layout = QHBoxLayout(); layout.setContentsMargins(0,0,0,0)
        self.add(layout, "Empty", "#FFFFE0"); self.add(layout, "Scanned", "#0000FF")
        self.add(layout, "Detected", "#00FFFF"); self.add(layout, "Planted", "#90EE90")
        self.add(layout, "Checked", "#FFD700")
        layout.addStretch(); self.setLayout(layout)
    def add(self, l, t, c):
        lbl = QLabel(); lbl.setFixedSize(15,15); lbl.setStyleSheet(f"background:{c}; border:1px solid grey")
        l.addWidget(lbl); l.addWidget(QLabel(t)); l.addSpacing(10)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AgriBot Command Center"); self.showMaximized()
        self.db = AgriBotDB(); self.ctrl = RosController(); self.ctrl.db_ref = self.db
        
        central = QWidget(); self.setCentralWidget(central); main_layout = QHBoxLayout(central) 
        left_widget = QWidget(); left_layout = QVBoxLayout(left_widget)
        head = QHBoxLayout(); head.addWidget(QLabel("<b>FIELD MAP</b>"))
        self.lbl_robot_pos = QLabel("Robot: X: 0.00m, Y: 0.00m"); self.lbl_robot_pos.setStyleSheet("font-weight: bold; color: blue; margin-left:15px")
        head.addWidget(self.lbl_robot_pos); head.addStretch()
        self.lbl_coords = QLabel("Cursor: 0,0"); head.addWidget(self.lbl_coords); left_layout.addLayout(head)
        left_layout.addWidget(LegendWidget())
        self.map_widget = MapWidget(self.db, self.ctrl); left_layout.addWidget(self.map_widget)
        main_layout.addWidget(left_widget, stretch=70)

        right_scroll = QScrollArea(); right_scroll.setWidgetResizable(True)
        right_widget = QWidget(); right_layout = QVBoxLayout(right_widget); right_layout.setAlignment(Qt.AlignTop)
        
        self.lbl_cam = QLabel("Camera Feed"); self.lbl_cam.setFixedSize(320, 240); self.lbl_cam.setStyleSheet("background: black; border: 2px solid grey; color: white"); self.lbl_cam.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(self.lbl_cam, alignment=Qt.AlignCenter)

        s_grp = QGroupBox("Seeder Status"); s_lay = QVBoxLayout()
        self.lbl_seed = QLabel("IDLE"); self.lbl_seed.setStyleSheet("font-weight: bold")
        self.bar_seed = QProgressBar(); self.bar_seed.setRange(-15, 20); self.bar_seed.setFormat("%v cm")
        s_lay.addWidget(self.lbl_seed); s_lay.addWidget(self.bar_seed); s_grp.setLayout(s_lay); right_layout.addWidget(s_grp)

        t_grp = QGroupBox("Tasks"); t_form = QFormLayout()
        self.n_points_spin = QSpinBox(); self.n_points_spin.setValue(3)
        btn_goto = QPushButton("Go To Point"); btn_goto.clicked.connect(lambda: self.set_mode("GOTO"))
        btn_plant = QPushButton("Plant Region"); btn_plant.clicked.connect(lambda: self.set_mode("PLANT"))
        btn_scan = QPushButton("Soil Scan Region"); btn_scan.clicked.connect(lambda: self.set_mode("SOIL_SCAN"))
        t_form.addRow("Samples:", self.n_points_spin); t_form.addRow(btn_goto); t_form.addRow(btn_plant); t_form.addRow(btn_scan); t_grp.setLayout(t_form); right_layout.addWidget(t_grp)

        db_grp = QGroupBox("Database"); db_layout = QHBoxLayout()
        btn_export = QPushButton("Export"); btn_export.clicked.connect(self.export_db)
        btn_reset = QPushButton("Reset"); btn_reset.setStyleSheet("color: red"); btn_reset.clicked.connect(self.reset_db)
        db_layout.addWidget(btn_export); db_layout.addWidget(btn_reset); db_grp.setLayout(db_layout); right_layout.addWidget(db_grp)

        m_grp = QGroupBox("Manual Control"); m_lay = QGridLayout()
        btns = [('▲', 0, 1, 2.0, 0.0), ('▼', 2, 1, -2.0, 0.0), ('◄', 1, 0, 0.0, 1.0), ('►', 1, 2, 0.0, -1.0), ('■', 1, 1, 0.0, 0.0)]
        for t, r, c, vx, vw in btns:
            b = QPushButton(t); b.setFixedSize(40,40)
            if t == '■': b.setStyleSheet("color: red"); b.clicked.connect(lambda: self.ctrl.set_manual_vel(0,0))
            else: b.pressed.connect(lambda vx=vx, vw=vw: self.ctrl.set_manual_vel(vx, vw)); b.released.connect(lambda: self.ctrl.set_manual_vel(0,0))
            m_lay.addWidget(b, r, c)
        
        m_lay.addWidget(QLabel("Drill"), 0, 3, 1, 1, Qt.AlignCenter)
        b_up = QPushButton("▲"); b_up.setFixedSize(40,40); b_dn = QPushButton("▼"); b_dn.setFixedSize(40,40)
        b_up.pressed.connect(lambda: self.ctrl.set_manual_seeder(1.0)); b_up.released.connect(lambda: self.ctrl.set_manual_seeder(0.0))
        b_dn.pressed.connect(lambda: self.ctrl.set_manual_seeder(-1.0)); b_dn.released.connect(lambda: self.ctrl.set_manual_seeder(0.0))
        m_lay.addWidget(b_up, 1, 3); m_lay.addWidget(b_dn, 2, 3); m_grp.setLayout(m_lay); right_layout.addWidget(m_grp)

        self.log_view = QTextEdit(); self.log_view.setMaximumHeight(100); self.log_view.setReadOnly(True); right_layout.addWidget(self.log_view)
        btn_estop = QPushButton("EMERGENCY STOP"); btn_estop.setStyleSheet("background:red; color:white; font-weight:bold; height: 40px")
        btn_estop.clicked.connect(self.e_stop); right_layout.addWidget(btn_estop)
        right_scroll.setWidget(right_widget); main_layout.addWidget(right_scroll, stretch=30)

        # Connections
        self.ctrl.update_odom.connect(self.map_widget.update_robot)
        self.ctrl.update_odom.connect(lambda x,y,t: self.lbl_robot_pos.setText(f"Robot: X: {x:.2f}m, Y: {y:.2f}m, Yaw: {math.degrees(t):.0f}°"))
        self.ctrl.update_cam.connect(lambda i: self.lbl_cam.setPixmap(QPixmap.fromImage(i).scaled(320, 240, Qt.KeepAspectRatio)))
        self.ctrl.update_path_viz.connect(self.map_widget.update_path)
        self.ctrl.update_sensors.connect(self.map_widget.update_sensors)
        self.ctrl.update_seeder.connect(lambda s, h: [self.lbl_seed.setText(s), self.bar_seed.setValue(int(h*100))])
        self.ctrl.point_reached.connect(self.map_widget.start_blink)
        self.ctrl.point_reached.connect(self.on_point_reached)
        self.ctrl.log_msg.connect(self.log)
        self.map_widget.mouse_moved_signal.connect(lambda x,y: self.lbl_coords.setText(f"Cursor: X:{x:.2f} Y:{y:.2f}"))
        
        # POPUP CONNECTION
        self.ctrl.show_popup.connect(lambda t, m: QMessageBox.information(self, t, m))
        
        self.ctrl.start()

    def log(self, m): self.log_view.append(f"[{datetime.now().strftime('%H:%M:%S')}] {m}"); sb = self.log_view.verticalScrollBar(); sb.setValue(sb.maximum())
    def set_mode(self, m): self.map_widget.mode = m; self.map_widget.setCursor(Qt.CrossCursor); self.log(f"Mode Set: {m}")
    def e_stop(self): self.ctrl.set_path([]); self.ctrl.state="IDLE"; self.ctrl.pub_vel.publish(Twist()); self.log("!!! EMERGENCY STOP !!!")

    def on_point_reached(self, x, y, t):
        if t == 'soil_scan':
            c = int((x - GRID_CONFIG['min_x']) / CELL_W); r = int((y - GRID_CONFIG['min_y']) / CELL_H)
            if 0 <= c < GRID_CONFIG['cols'] and 0 <= r < GRID_CONFIG['rows']:
                self.db.update_cell(c, r, 'soil_analyzed'); self.log(f"Soil Sampled at Grid[{c},{r}]")

    def export_db(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export Database", "", "CSV Files (*.csv)")
        if path:
            try:
                cursor = self.db.conn.cursor(); cursor.execute("SELECT * FROM grid_cells"); rows = cursor.fetchall()
                with open(path, 'w', newline='') as f:
                    writer = csv.writer(f); writer.writerow(["ID", "Grid X", "Grid Y", "State", "Batch ID", "Timestamp", "Data"]); writer.writerows(rows)
                self.log(f"Exported to {path}")
            except Exception as e: self.log(f"Export Error: {e}")

    def reset_db(self):
        if QMessageBox.question(self, 'Reset DB', "Clear all data?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            try: self.db.conn.cursor().execute("DELETE FROM grid_cells"); self.db.conn.commit(); self.map_widget.update(); self.log("Database Cleared.")
            except Exception as e: self.log(f"Reset Error: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv); win = MainWindow(); win.show(); sys.exit(app.exec_())
import math
import cv2
import numpy as np
import config

class PlantDetector:
    def __init__(self):
        # Camera Intrinsics & Extrinsics
        self.IMG_W = 640
        self.IMG_H = 480
        self.FOV_H = 1.047 # Horizontal Field of View (radians)
        
        # Pinhole Camera Math
        self.FOCAL_LENGTH = (self.IMG_W / 2) / math.tan(self.FOV_H / 2) 
        
        # Physical Object Property
        self.REAL_PLANT_WIDTH = 0.20 # meters
        
        # Camera Transform (Relative to Base Link)
        # Matches Xacro: x=0.45, y=-0.0334
        self.CAM_OFF_X = 0.45
        self.CAM_OFF_Y = -0.0334

    def process_image(self, cv_image, robot_pose):
        """
        Process the raw CV image to detect green plants and calculate their world coordinates.
        
        Args:
            cv_image: The BGR image from OpenCV/ROS.
            robot_pose: Tuple (rx, ry, ryaw) of the robot's current world pose.
            
        Returns:
            processed_img: Image with debug drawings (bounding boxes, text).
            detections: List of tuples [(world_x, world_y), ...] for each detected plant.
        """
        rx, ry, ryaw = robot_pose
        detections = []
        
        # Pre-process
        blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Green Mask (H: 30-90)
        mask = cv2.inRange(hsv, np.array([30, 40, 40]), np.array([90, 255, 255]))
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Find Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            if cv2.contourArea(c) < 500: 
                continue
                
            (x, y, w, h) = cv2.boundingRect(c)
            
            # --- TRIGONOMETRY & PINHOLE MATH ---
            # Distance estimation based on known object width
            d = (self.FOCAL_LENGTH * self.REAL_PLANT_WIDTH) / w
            
            # Filter detections that are too far (noisy)
            if d > 3.0: 
                continue 

            # Angle relative to camera center
            cx = x + (w // 2)
            offset_px = (self.IMG_W / 2) - cx 
            alpha = math.atan2(offset_px, self.FOCAL_LENGTH)

            # Transform from Camera Frame to World Frame
            total_yaw = ryaw + alpha
            cos_r = math.cos(ryaw)
            sin_r = math.sin(ryaw)
            
            # 1. Camera World Position
            cam_wx = rx + (self.CAM_OFF_X * cos_r - self.CAM_OFF_Y * sin_r)
            cam_wy = ry + (self.CAM_OFF_X * sin_r + self.CAM_OFF_Y * cos_r)
            
            # 2. Plant World Position
            plant_wx = cam_wx + (d * math.cos(total_yaw))
            plant_wy = cam_wy + (d * math.sin(total_yaw))

            detections.append((plant_wx, plant_wy))
            
            # Debug UI
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(cv_image, f"{d:.1f}m", (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

        return cv_image, detections

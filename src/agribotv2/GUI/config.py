# config.py
import math
import os

# ==========================================
# WORLD & GRID SETTINGS
# ==========================================
# World Bounds (Meters)
WORLD_MIN_X = -2.0
WORLD_MAX_X = 12.0   
WORLD_MIN_Y = -7.0
WORLD_MAX_Y = 7.0    

# Grid Configuration - 2m spacing between plants
# Plants at X: 2, 4, 6 and Y: -2, 0, 2 (2m apart)
# Grid cells are 2m x 2m with plants at cell centers
GRID_CELL_SIZE = 2.0  # Each cell is 2m x 2m (square)
GRID_CONFIG = {
    'min_x': 1.0,   'min_y': -3.0,   # Grid starts 1m before first plant
    'max_x': 7.0,   'max_y': 3.0,    # Grid ends 1m after last plant
    'rows': 3,      'cols': 3        # 3 columns (X), 3 rows (Y)
}

# Calculated Cell Dimensions (should be 2.0m each for square cells)
CELL_W = (GRID_CONFIG['max_x'] - GRID_CONFIG['min_x']) / GRID_CONFIG['cols']  # 2.0m
CELL_H = (GRID_CONFIG['max_y'] - GRID_CONFIG['min_y']) / GRID_CONFIG['rows']  # 2.0m

# ==========================================
# ROBOT & SENSOR SETTINGS
# ==========================================
ROBOT_LENGTH = 1.2 
ROBOT_WIDTH = 0.7   

# Sensor Offsets (Relative to Robot Center)
# US2, US3: FRONT sensors
# US4: LEFT side sensor  
# US1: RIGHT side sensor
SENSOR_OFFSETS = {
    'us1': {'x': 0.138, 'y': -0.422, 'yaw': -0.6283},  # Right side
    'us2': {'x': 0.45,  'y': -0.282, 'yaw': 0.0},      # Front-right (straight)
    'us3': {'x': 0.45,  'y': 0.233,  'yaw': -0.3725},  # Front-left (angled)
    'us4': {'x': 0.138, 'y': 0.373,  'yaw': 0.6283}    # Left side
}
US_FOV = math.radians(60)
US_MAX_RANGE = 4.0

# ==========================================
# ASSETS
# ==========================================
# Use absolute path based on this file's location
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BG_PATH = os.path.join(_SCRIPT_DIR, "sandbackground.jpg")
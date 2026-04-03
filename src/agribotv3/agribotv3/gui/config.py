"""Shared GUI configuration for AgriBot v3."""
from __future__ import annotations

import csv
import math
import os
from typing import Final

# World bounds (metres)
WORLD_MIN_X: Final[float] = -2.0
WORLD_MAX_X: Final[float] = 12.0
WORLD_MIN_Y: Final[float] = -7.0
WORLD_MAX_Y: Final[float] = 7.0

# Legacy farm grid (3x3)
GRID_CONFIG: Final[dict[str, float | int]] = {
    'min_x': 1.0,
    'min_y': -3.0,
    'max_x': 7.0,
    'max_y': 3.0,
    'rows': 3,
    'cols': 3,
}
CELL_W: Final[float] = (GRID_CONFIG['max_x'] - GRID_CONFIG['min_x']) / GRID_CONFIG['cols']
CELL_H: Final[float] = (GRID_CONFIG['max_y'] - GRID_CONFIG['min_y']) / GRID_CONFIG['rows']

# Vehicle dimensions
ROBOT_LENGTH: Final[float] = 1.2
ROBOT_WIDTH: Final[float] = 0.7

# Ultrasonic geometry and rendering
SENSOR_OFFSETS: Final[dict[str, dict[str, float]]] = {
    'us1': {'x': 0.138, 'y': -0.422, 'yaw': -0.6283},
    'us2': {'x': 0.45, 'y': -0.282, 'yaw': 0.0},
    'us3': {'x': 0.45, 'y': 0.233, 'yaw': -0.3725},
    'us4': {'x': 0.138, 'y': 0.373, 'yaw': 0.6283},
}
US_FOV: Final[float] = math.radians(60)
US_MAX_RANGE: Final[float] = 4.0

# Visual palette
STATE_COLOURS: Final[dict[str, str]] = {
    'empty': '#d4c89a',
    'scanned': '#6ec6ff',
    'detected': '#ffa726',
    'planted': '#66bb6a',
    'checked': '#ab47bc',
}

PLANT_STATE_COLOURS: Final[dict[str, str]] = {
    'not_planted': '#808080',
    'detected': '#FFD700',
}

BATCH_COLOURS: Final[list[str]] = [
    '#00CED1', '#FF6B35', '#7B68EE', '#32CD32', '#FF69B4',
    '#20B2AA', '#FF4500', '#9370DB', '#E91E63', '#00BCD4',
]

# Mission defaults
SOIL_SCAN_DWELL: Final[float] = 2.0
SOIL_SCAN_N_DEFAULT: Final[int] = 3
PLANT_CELL_SIZE: Final[float] = 1.0

# Future Nav2 integration points for motion stack/obstacle avoidance modules
NAV2_TOPICS: Final[dict[str, str]] = {
    'cmd_vel': '/cmd_vel',
    'odometry': '/odometry/filtered',
    'global_plan': '/plan',
    'goal_action': 'navigate_to_pose',
    'waypoints_action': 'follow_waypoints',
}

_DATA_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'data')
SEEDS_CSV = os.path.join(_DATA_DIR, 'seeds.csv')
PLANTS_CSV = os.path.join(_DATA_DIR, 'plants.csv')


def get_batch_colour(batch_id: str) -> str:
    return BATCH_COLOURS[hash(batch_id) % len(BATCH_COLOURS)]


def _load_csv_rows(path: str) -> list[dict]:
    try:
        with open(path, encoding='utf-8') as file:
            return list(csv.DictReader(file))
    except FileNotFoundError:
        return []


def load_plants_csv() -> list[dict]:
    return _load_csv_rows(PLANTS_CSV)


def load_seeds_csv() -> list[dict]:
    return _load_csv_rows(SEEDS_CSV)

"""World, grid, robot & sensor constants — no ROS / GUI dependencies."""
import math
import os
import csv
import csv

# ── World bounds (metres) ────────────────────────────────────
WORLD_MIN_X = -2.0
WORLD_MAX_X = 12.0
WORLD_MIN_Y = -7.0
WORLD_MAX_Y = 7.0

# ── Grid ─────────────────────────────────────────────────────
GRID_CELL_SIZE = 2.0

GRID_CONFIG = {
    'min_x': 1.0,  'min_y': -3.0,
    'max_x': 7.0,  'max_y':  3.0,
    'rows': 3,     'cols': 3,
}

CELL_W = (GRID_CONFIG['max_x'] - GRID_CONFIG['min_x']) / GRID_CONFIG['cols']  # 2.0
CELL_H = (GRID_CONFIG['max_y'] - GRID_CONFIG['min_y']) / GRID_CONFIG['rows']  # 2.0

# ── Robot ────────────────────────────────────────────────────
ROBOT_LENGTH = 1.2
ROBOT_WIDTH  = 0.7

# ── Ultrasonic sensors ──────────────────────────────────────
SENSOR_OFFSETS = {
    'us1': {'x':  0.138, 'y': -0.422, 'yaw': -0.6283},   # right
    'us2': {'x':  0.45,  'y': -0.282, 'yaw':  0.0},       # front-right
    'us3': {'x':  0.45,  'y':  0.233, 'yaw': -0.3725},    # front-left
    'us4': {'x':  0.138, 'y':  0.373, 'yaw':  0.6283},    # left
}
US_FOV       = math.radians(60)
US_MAX_RANGE = 4.0

# ── Colours (hex for web UI) ────────────────────────────────
STATE_COLOURS = {
    'empty':    '#d4c89a',
    'scanned':  '#6ec6ff',
    'detected': '#ffa726',
    'planted':  '#66bb6a',
    'checked':  '#ab47bc',
}

# ── Plant cell states (from plants.csv) ─────────────────────────
PLANT_STATE_COLOURS = {
    'not_planted': '#808080',
    'detected':    '#FFD700',
    # 'planted' → colour comes from batch
}

# ── Batch palette (deterministic: hash(batch_id) % len) ─────────
BATCH_COLOURS = [
    '#00CED1', '#FF6B35', '#7B68EE', '#32CD32',
    '#FF69B4', '#20B2AA', '#FF4500', '#9370DB',
    '#E91E63', '#00BCD4',
]


def get_batch_colour(batch_id: str) -> str:
    return BATCH_COLOURS[hash(batch_id) % len(BATCH_COLOURS)]


# ── CSV data paths ───────────────────────────────────────────────
_DATA_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'data')
SEEDS_CSV  = os.path.join(_DATA_DIR, 'seeds.csv')
PLANTS_CSV = os.path.join(_DATA_DIR, 'plants.csv')


def load_plants_csv() -> list[dict]:
    """Return list of plant dicts from plants.csv (empty list if missing)."""
    try:
        with open(PLANTS_CSV) as f:
            return list(csv.DictReader(f))
    except FileNotFoundError:
        return []


def load_seeds_csv() -> list[dict]:
    """Return list of seed dicts from seeds.csv (empty list if missing)."""
    try:
        with open(SEEDS_CSV) as f:
            return list(csv.DictReader(f))
    except FileNotFoundError:
        return []


# ── Soil scan ────────────────────────────────────────────────────
SOIL_SCAN_DWELL     = 2.0   # seconds to dwell at each sample point
SOIL_SCAN_N_DEFAULT = 3     # default number of random sample points

# ── Plant grid cell size (m) — individual plant slot ─────────────
PLANT_CELL_SIZE = 1.0

# ── Plant cell states (from plants.csv) ─────────────────────────
PLANT_STATE_COLOURS = {
    'not_planted': '#808080',
    'detected':    '#FFD700',
    # 'planted' → colour comes from batch
}

# ── Batch palette (deterministic: hash(batch_id) % len) ─────────
BATCH_COLOURS = [
    '#00CED1', '#FF6B35', '#7B68EE', '#32CD32',
    '#FF69B4', '#20B2AA', '#FF4500', '#9370DB',
    '#E91E63', '#00BCD4',
]


def get_batch_colour(batch_id: str) -> str:
    return BATCH_COLOURS[hash(batch_id) % len(BATCH_COLOURS)]


# ── CSV data paths ───────────────────────────────────────────────
_DATA_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'data')
SEEDS_CSV  = os.path.join(_DATA_DIR, 'seeds.csv')
PLANTS_CSV = os.path.join(_DATA_DIR, 'plants.csv')


def load_plants_csv() -> list[dict]:
    """Return list of plant dicts from plants.csv (empty list if missing)."""
    try:
        with open(PLANTS_CSV) as f:
            return list(csv.DictReader(f))
    except FileNotFoundError:
        return []


def load_seeds_csv() -> list[dict]:
    """Return list of seed dicts from seeds.csv (empty list if missing)."""
    try:
        with open(SEEDS_CSV) as f:
            return list(csv.DictReader(f))
    except FileNotFoundError:
        return []


# ── Soil scan ────────────────────────────────────────────────────
SOIL_SCAN_DWELL    = 2.0   # seconds to dwell at each sample point
SOIL_SCAN_N_DEFAULT = 3    # default number of random sample points

# ── Plant grid cell size (m) — individual plant slot ─────────────
PLANT_CELL_SIZE = 1.0
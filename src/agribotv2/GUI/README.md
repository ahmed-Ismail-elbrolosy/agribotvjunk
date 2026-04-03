# AgriBot GUI Control System


> **Note:** For the newer NiceGUI dashboard (`agribotv3`), the GUI can run standalone and wait for ROS topics without crashing when publishers are absent.

This project is a modular GUI application for controlling an agricultural robot using **ROS 2** and **PyQt5**. It handles path planning, manual control, sensor visualization, and database management for farming tasks like planting and soil scanning.

---

## 🌱 How Each Function Works

### 1. Soil Scanning (`SOIL_SCAN` Job)

**Purpose:** Analyze soil at multiple points in a selected region to determine nutrient levels and conditions.

**Workflow:**
1. User selects a region on the map by clicking and dragging
2. The system generates N sample points within the region (configured via UI spinner)
3. Robot navigates to each point using A* path planning
4. At each point:
   - Robot stops and lowers the seeder mechanism (`SEEDER_ACTION` state)
   - Simulated soil sensors read 7 parameters:
     - Nitrogen (N) in ppm
     - Phosphorus (P) in ppm  
     - Potassium (K) in ppm
     - pH level (5.5-8.0)
     - Moisture percentage (15-60%)
     - Temperature in Celsius (15-35°C)
     - Electrical Conductivity (EC) in mS/cm
   - Data is saved to database with cell status `soil_analyzed`
   - Cell turns **blue** on the map
5. After all points are scanned, a summary report popup shows averages

**Key Code Locations:**
- Point generation: `map_widget.py` → `mouseReleaseEvent()` with mode `SOIL_SCAN`
- Soil reading: `controller.py` → `control_loop()` in `SEEDER_ACTION` state
- Summary report: `controller.py` → `_show_soil_scan_summary()`

---

### 2. Planting (`PLANT` Job)

**Purpose:** Plant seeds at grid cell centers in a selected region.

**Workflow:**
1. User selects a region on the map
2. System generates waypoints at the center of each grid cell in the region
3. Robot navigates to each cell center
4. At each point:
   - Seeder mechanism lowers into the ground
   - Plant is "placed" (simulated)
   - Cell status updated to `planted` with batch_id (timestamp-based)
   - Cell turns **green** on the map (color varies by batch)
   - Plant added as obstacle for future path planning

**Key Code Locations:**
- Waypoint generation: `map_widget.py` → region selection with mode `PLANT`
- Planting action: `controller.py` → `SEEDER_ACTION` state when `type == 'plant'`
- Database update: `database.py` → `update_cell()`

---

### 3. Vision-Based Plant Detection

**Purpose:** Detect existing plants using the robot's camera and mark them on the map.

**Workflow:**
1. Camera feed received via ROS topic `/camera/image_raw`
2. Image processing pipeline:
   - Convert BGR to HSV color space
   - Apply green color threshold (H: 35-85, S: 40-255, V: 40-255)
   - Morphological operations (open/close) to clean noise
   - Find contours of green regions
3. For each detected plant:
   - Calculate ground distance using geometric projection:
     ```
     distance = camera_height / tan(camera_tilt + pixel_angle)
     ```
   - Camera parameters: height=0.337m, tilt=45°, max detection=1.0m
   - Transform pixel coordinates to world coordinates
   - Snap to nearest grid cell
4. Mark cell as `detected` (cyan color) in database
5. Add plant as obstacle for path planning (0.15m radius)

**Key Code Locations:**
- Vision node: `AgriBotKin/vision_node.py` → `PlantDetector` class
- Controller callback: `controller.py` → `vision_detection_cb()`
- Camera config: `robot.xacro` → camera macro with 45° tilt

---

### 4. Path Planning & Following

**Purpose:** Navigate the robot from current position to goal while avoiding obstacles.

**Path Generation (A* Algorithm):**
1. Create occupancy grid from world bounds (resolution: 0.15m)
2. Mark obstacles with robot radius inflation (0.6m)
3. A* search with 8-directional movement
4. Path optimization:
   - Line-of-sight simplification (skip redundant waypoints)
   - Bezier curve smoothing at corners
   - Max turn angle constraint: 25° per segment

**Path Following:**
1. Reference point: 0.8m ahead of robot center (magenta dot on map)
2. For each waypoint:
   - Calculate angle to target
   - PD controller for angular velocity
   - Speed control based on:
     - Turn angle (slow when turning)
     - Obstacle proximity from ultrasonic sensors
3. Waypoint tolerance: 0.15m (reached when within this distance)

**Key Code Locations:**
- Planner: `fast_planner.py` → `FastPlanner` class
- Path following: `controller.py` → `FOLLOW_PATH` state
- Obstacle addition: `controller.py` → `add_and_validate_obstacle()`

---

### 5. Obstacle Avoidance

**Purpose:** Avoid collisions with detected obstacles and plants.

**Sensor-Based Avoidance:**
1. 4 ultrasonic sensors (US1-US4) around robot perimeter
2. When sensor detects object < 1.0m:
   - Calculate obstacle world position
   - Add to planner grid with 1.1m clearance radius
   - Trigger path replan if current path is blocked

**Plant Avoidance:**
- All `planted` and `detected` cells loaded as obstacles at job start
- Obstacle radius: 0.15m per plant + 0.6m robot inflation = 0.75m clearance

**Key Code Locations:**
- Sensor callback: `controller.py` → `us_cb()`
- Obstacle registration: `controller.py` → `add_and_validate_obstacle()`
- Path replan: Triggered when obstacle within 1.2m of path

---

### 6. Manual Control

**Purpose:** Direct teleoperation of robot via UI buttons.

**Controls:**
- Forward/Backward: Linear velocity (±1.0 m/s)
- Left/Right: Angular velocity (±1.0 rad/s)
- Seeder Up/Down: Manual seeder height control

**Key Code Location:**
- `controller.py` → `set_manual_vel()`, `set_manual_seeder()`
- State changes to `MANUAL` during active control

---

### 7. State Machine

The robot operates on a state machine with the following states:

| State | Description |
|-------|-------------|
| `IDLE` | No active task, robot stopped |
| `NEXT_GLOBAL_POINT` | Pop next waypoint from queue, plan path |
| `ROTATING` | Rotate in place to face first path waypoint |
| `FOLLOW_PATH` | Follow planned path to current goal |
| `SEEDER_ACTION` | Perform soil scan or planting at current location |
| `MANUAL` | User controlling via buttons/joystick |

---

### 8. Seeder Mechanism

**Purpose:** Physical mechanism for soil interaction (drilling/planting).

**States:**
1. `IDLE` - Seeder at rest position (0.20m height)
2. `LOWERING` - Moving down toward ground
3. `WORKING` - Drilling into soil (goes to -0.15m)
4. `WAITING` - Pause for 1.5 seconds at depth
5. `RAISING` - Moving back up to rest position

**Key Code Location:**
- `controller.py` → `SEEDER_ACTION` state in `control_loop()`

---

## 📂 File Structure

| File | Purpose |
|------|---------|
| `main.py` | Entry point, launches Qt application |
| `config.py` | World bounds, grid config, robot dimensions |
| `gui.py` / `dashboard.py` | UI layout and button connections |
| `controller.py` | ROS node, state machine, sensor processing |
| `map_widget.py` | Map visualization, mouse interaction |
| `fast_planner.py` | A* path planning with smoothing |
| `database.py` | CSV-based persistence for grid cells |
| `vision.py` | Local OpenCV plant detection (optional) |

---

## 🎨 Map Color Legend

| Status | Color | Description |
|--------|-------|-------------|
| Empty | Light Yellow | Unvisited cell |
| Scanned | Blue | Soil analyzed |
| Detected | Cyan | Plant detected by camera |
| Planted | Green (varies) | Robot-planted (unique per batch) |
| Checked | Gold | Plant health verified |

---

## 📡 ROS 2 Topics

### Published:
- `/cmd_vel` (Twist) - Robot velocity commands
- `/vision/robot_pose` (String/JSON) - Robot position for vision node

### Subscribed:
- `/camera/image_raw` (Image) - Camera feed
- `/ultra/us1..us4` (Range) - Ultrasonic sensor data
- `/world/SinaiAgri/dynamic_pose/info` (Pose) - Robot odometry from Gazebo
- `/vision/plant_detections` (String/JSON) - Detected plants from vision node
- `/vision/status` (String/JSON) - Vision node status

---

## 🚀 How to Run

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/projects/src/agribotv2install/setup.bash

# Run full simulation with vision
ros2 launch AgriBotKin agribot_full.launch.py

# Or run GUI only (connects to existing simulation)
cd ~/projects/src/agribotv2AgriBotKin/GUI
python3 main.py
```

---

## 🔧 Configuration Parameters

Edit `config.py` to change:

```python
# World boundaries (meters)
WORLD_MIN_X = -2.0
WORLD_MAX_X = 12.0
WORLD_MIN_Y = -7.0
WORLD_MAX_Y = 7.0

# Grid settings
GRID_CELL_SIZE = 2.0  # 2m x 2m cells

# Robot dimensions
ROBOT_LENGTH = 0.9
ROBOT_WIDTH = 0.86
```

Edit `fast_planner.py` for path planning:

```python
ROBOT_FRONT_OFFSET = 0.8  # Reference point ahead of center
MAX_TURN_ANGLE = 25°      # Maximum turn per path segment
resolution = 0.15         # Grid cell size for planning
robot_radius = 0.6        # Obstacle inflation radius
```

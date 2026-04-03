# AgriBot v3 GUI — Current Behavior (as implemented)

This document explains how the GUI in `src/agribotv3/agribotv3/gui` currently works.

## 1) Entry points and startup

- ROS entrypoint is `gui = agribotv3.gui.app:main` in `setup.py`.
- You can run either:
  - `ros2 run agribotv3 gui`
  - `python3 -m agribotv3.gui.app`
- `main()` initializes `rclpy` (if not already initialized), ensures singleton instances are created, then starts NiceGUI at:
  - `host=0.0.0.0`
  - `port=8080`
  - page path `/`

## 2) Main modules and responsibilities

### `app.py`
- Builds and runs the NiceGUI web dashboard.
- Exposes MJPEG stream endpoint `/video_feed` for camera display.
- Renders map, robot pose, ultrasonic cones, nav path, goals, mission waypoints, and plant/grid overlays.
- Handles UI interactions (click-to-nav, area selection, E-STOP, mission actions, manual controls).
- Runs periodic UI refresh timer (every 0.20 s / 5 Hz).

### `controller.py`
- Owns ROS 2 integration via a dedicated daemon thread.
- Spawns internal node `_ControllerNode` and spins with `rclpy.spin_once(..., timeout_sec=0.05)`.
- Subscribes to:
  - `/odometry/filtered` (pose/velocity)
  - `/ultra/us1..us4` (range sensors)
  - `/camera/image_raw` (raw image bytes)
  - `/plan` (Nav2 path)
  - `/vision/plant_detections` (JSON text)
- Publishes:
  - `/cmd_vel` (manual velocity commands)
- Uses Nav2 actions:
  - `navigate_to_pose` (`NavigateToPose`)
  - `follow_waypoints` (`FollowWaypoints`)
- Maintains shared `RobotState` used by GUI.

### `database.py`
- CSV-backed, thread-safe storage helper.
- Manages:
  - `grid_cells.csv`
  - `seeds.csv`
  - `soil_scans.csv`
- Creates default files (and starter seed rows) if missing.

### `config.py`
- Static constants and helpers for world bounds, grid geometry, robot dimensions, ultrasonic setup, color mapping, and CSV loading (`plants.csv`, `seeds.csv`).

## 3) Singleton lifecycle

`app.py` uses two process-level singletons:
- `ctrl: RosController | None`
- `db: AgriBotDB | None`

`_ensure_singletons()` does:
1. Create and start `RosController()` once.
2. Resolve data directory from package share (`ament_index_python`) when available; fallback to local relative path.
3. Create `AgriBotDB(data_dir)` once.

## 4) Camera feed path (`/video_feed`)

- Camera bytes are received from ROS image messages into `RobotState`.
- Endpoint loop checks frame validity (size matches width/height/channels).
- Converts bytes -> NumPy array -> OpenCV JPEG (`quality=70`).
- Streams as multipart MJPEG with short sleep (`0.05 s`) between attempts.

## 5) Map visualization behavior

`_build_map_fig(...)` composes a Plotly figure with:
- Fixed world bounds and dark theme.
- Dual-weight grid lines (0.5 m minor, 1.0 m major).
- Plant-cell overlays from `plants.csv`:
  - cell centers snapped by `PLANT_CELL_SIZE`
  - deduplicated by center coordinate
  - color rules:
    - `detected` -> `PLANT_STATE_COLOURS['detected']`
    - `not_planted` -> `PLANT_STATE_COLOURS['not_planted']`
    - otherwise batch color via `get_batch_colour(batch_id)`
- Legacy 3×3 DB grid overlay from `grid_cells.csv` states.
- Optional selected area rectangle.
- Nav2 planned path (`/plan`) polyline.
- Mission waypoints and active/completed coloring.
- Current nav goal marker (red X).
- 4 ultrasonic sensor cones using robot pose + sensor offsets + FOV.
- Robot body polygon, heading arrow, and center marker.
- Transparent heatmap layer for reliable click capture.

## 6) UI layout and controls

Single-page UI (`@ui.page('/')`) includes:
- Header with nav status badge.
- Left panel:
  - live position strip
  - interactive map
  - camera feed (`<img src="/video_feed">`)
- Right panel:
  - E-STOP and Cancel Nav
  - coordinate inputs + GO button
  - manual drive (buttons + WASD keyboard)
  - ultrasonic values + bars
  - mission controls:
    - Soil Scan (area-based random sample waypoints)
    - Plant Region (area-based plant-cell centers)
    - seed selector loaded from `seeds.csv`
  - waypoint list
  - activity log
  - map legend

## 7) Interaction model

### Click behavior on map
- **Normal mode**: clicking map sets X/Y inputs and immediately sends single Nav2 goal.
- **Area mode** (`soil_scan_area` or `plant_area`): two clicks define a rectangle.

### Soil Scan flow
1. User clicks **Soil Scan** (with sample count `n`).
2. User selects two corners on map.
3. GUI generates `n` random waypoints inside rectangle.
4. Async mission `ctrl.run_soil_scan(...)`:
   - for each waypoint: `navigate_to(...)`, wait `3.0 + dwell`, log activity.
   - mission can be canceled via E-STOP/Cancel.

### Plant Region flow
1. User clicks **Plant Region**.
2. User selects rectangle.
3. GUI scans loaded plant list and collects unique plant-cell centers inside rectangle.
4. Async mission `ctrl.run_planting(...)` visits each center.

## 8) Periodic updates (5 Hz)

Timer callback updates:
- Position labels (x, y, yaw).
- Operation/nav badges (idle, navigating, scanning, planting, stopped/cancelled).
- Ultrasonic text + color + fill percentage bars.
- Waypoint list panel.
- Full map redraw using latest shared state.

## 9) Safety and cancellation behavior

- **E-STOP** calls `ctrl.abort_mission()` and `ctrl.stop()`.
- **Cancel Nav** calls `ctrl.abort_mission()`.
- `abort_mission()` sets mission cancel flag, clears operation, cancels active nav action, and stops robot.

## 10) Current persistence behavior

- GUI reads static `plants.csv` and `seeds.csv` from data directory at page load.
- `AgriBotDB` provides methods for grid cell updates, seed CRUD, and soil scan records.
- In `app.py`, DB is currently used for reading all grid cells for map overlay; mission routines currently log actions but do not write new soil scan entries.

## 11) Notable implementation notes (current state)

- `config.py` contains duplicate blocks (imports/constants/functions repeated), but behavior still resolves due to later redefinitions matching intent.
- Seed dropdown uses CSV loader output keyed by `seed_id`/`name` fields from `seeds.csv` as currently expected by GUI.
- UI is designed for local web access (`http://localhost:8080`) but binds all interfaces (`0.0.0.0`).

---

If you want, the next step can be a second markdown file that adds sequence diagrams (startup, click-to-nav, soil scan, and planting mission) for easier onboarding.

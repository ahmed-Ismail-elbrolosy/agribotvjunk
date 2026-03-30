# CLAUDE.md — AgriBot v3 GUI LLM Context

This file is a fast, high-signal guide for LLM agents working in `src/agribotv3/agribotv3/gui/`.
Use this first to avoid scanning the full codebase.

---

## 1) What this module is

`agribotv3.gui` is a **NiceGUI web dashboard** (port `8080`) that overlays:
- live robot state (pose, velocity, nav status),
- Nav2 path + goal,
- mission tools (soil scan + planting),
- camera MJPEG stream,
- ultrasonic visualization,
- CSV-backed farm data.

It is intentionally a **thin orchestration layer** around ROS 2 Nav2 + simple CSV persistence.

---

## 2) File-level map (read order)

1. `app.py` — UI, map drawing, user interactions, timers, and mission triggers.
2. `controller.py` — ROS node in daemon thread, action clients, mission execution state.
3. `database.py` — CSV persistence for grid cells, seeds, and soil scans.
4. `config.py` — world/grid constants, color palettes, CSV loaders, sensor geometry.
5. `__init__.py` — package marker.

If you only have time for one file, start with `app.py`.

---

## 3) Runtime architecture

### Main ownership
- **UI/event loop**: NiceGUI in `app.py`.
- **ROS spin loop**: daemon thread in `RosController._spin()` (`controller.py`).
- **Shared mutable state**: `RobotState` dataclass instance (`controller.py`) read by UI and written by ROS callbacks.

### Critical invariant
- UI never blocks on ROS spinning.
- ROS callbacks update `RobotState`; periodic UI timer (`0.20s`) redraws from that state.

### Singleton pattern
`app.py` uses module-level singletons:
- `ctrl: RosController | None`
- `db: AgriBotDB | None`
initialized by `_ensure_singletons()`.

---

## 4) ROS interfaces in use

### Subscriptions
- `/odometry/filtered` (`nav_msgs/Odometry`) → pose/velocity.
- `/ultra/us1..us4` (`sensor_msgs/Range`) → ultrasonic distances.
- `/camera/image_raw` (`sensor_msgs/Image`) → raw camera frame.
- `/plan` (`nav_msgs/Path`) → planned path polyline.
- `/vision/plant_detections` (`std_msgs/String`) → JSON string payload.

### Publisher
- `/cmd_vel` (`geometry_msgs/Twist`) for manual drive and stop.

### Nav2 actions
- `navigate_to_pose` (`NavigateToPose`) for single target.
- `follow_waypoints` (`FollowWaypoints`) for multi-point routes.

---

## 5) UI behavior cheatsheet

### Map click behavior (`_on_map_click`)
- Default mode: clicking map sends immediate nav goal.
- Soil scan mode: 2 clicks define rectangle; random sample waypoints are generated.
- Plant region mode: 2 clicks define rectangle; plant cell centers are extracted from `plants.csv`.

### Mission operations
- `soil_scan`: visit each generated waypoint, dwell N seconds.
- `planting`: visit each selected plant-cell center.
- Cancellation: `abort_mission()` sets `mission_cancel`, cancels nav, and stops robot.

### Refresh loop
`ui.timer(0.20, _tick)` updates:
- robot readouts,
- status badges,
- ultrasonic bars,
- waypoint list,
- full map re-render.

---

## 6) Data model summary

### `RobotState` (controller-owned)
Key fields:
- kinematics: `x, y, yaw, vx, wz`
- sensors: `us`, camera (`cam_data`, `cam_w`, `cam_h`, `cam_encoding`)
- nav: `nav_active`, `nav_status`, `path`, `goal_x`, `goal_y`, `has_goal`
- mission: `operation`, `mission_waypoints`, `mission_wp_idx`, `mission_cancel`

### CSV persistence (`AgriBotDB`)
Files in data dir:
- `grid_cells.csv`
- `seeds.csv`
- `soil_scans.csv`

Database class is lock-protected (`threading.Lock`) and safe for concurrent GUI/mission use.

---

## 7) Coordinate systems

- **World** coordinates are meters in map frame.
- **Grid** coordinates are legacy 3x3 cell indices from `GRID_CONFIG`.
- Helpers in `controller.py`:
  - `world_to_grid(wx, wy)`
  - `grid_to_world(col, row)`

Plant-cell visualization uses `PLANT_CELL_SIZE` (1.0m) and quantizes plant points to cell centers.

---

## 8) Known quirks / gotchas

1. `config.py` currently has duplicated blocks (`PLANT_STATE_COLOURS`, `BATCH_COLOURS`, CSV helpers) — avoid copying this pattern.
2. `run_soil_scan` / `run_planting` use fixed `asyncio.sleep(...)` timing rather than waiting for explicit Nav2 success per waypoint.
3. Camera MJPEG endpoint assumes raw image byte layout based on encoding and channel count.
4. `app.py` imports `world_to_grid`/`grid_to_world` but current UI mostly operates directly in world coords.

---

## 9) Safe edit guidelines for LLM agents

When changing behavior:
1. Keep ROS spinning in controller thread; do not move `rclpy.spin_once` into UI loop.
2. Preserve `RobotState` as the cross-thread contract.
3. Avoid blocking calls in UI callbacks.
4. Prefer adding helper functions over expanding huge closures in `index()`.
5. If adding persistent fields, update CSV headers + read/write paths consistently in `database.py`.
6. Keep map rendering idempotent (rebuild from state each tick).

When adding features:
- Add constants to `config.py`.
- Add ROS I/O + state wiring in `controller.py`.
- Add UI/control flow in `app.py`.
- Add storage schema in `database.py` only if persistence is needed.

---

## 10) Local run & debug quick commands

From workspace root:

```bash
python3 -m agribotv3.gui.app
```

ROS entrypoint (if package installed):

```bash
ros2 run agribotv3 gui
```

Open:

```text
http://localhost:8080
```

---

## 11) Fast task routing for future LLM sessions

- "Map visuals wrong" -> `app.py` (`_build_map_fig`, `_tick`).
- "Robot not moving manually" -> `controller.py` (`send_vel`, `/cmd_vel`).
- "Nav2 goal issues" -> `controller.py` (`navigate_to`, action callbacks).
- "Soil scan/plant mission logic" -> `app.py` handlers + `controller.py` mission coroutines.
- "Seed/scan data issues" -> `database.py` + `config.py` CSV paths.
- "Theme/layout tweaks" -> `app.py` CSS block + card/button sections.

---

## 12) Definition of done for edits in this folder

Before finishing:
1. App imports cleanly.
2. No syntax errors in edited file(s).
3. Any new UI element is updated by `_tick` if state-driven.
4. Any new mission state is reset on cancel/complete.
5. Docs in this file updated when architecture changes.


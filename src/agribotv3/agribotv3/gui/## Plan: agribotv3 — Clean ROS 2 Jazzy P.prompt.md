## Plan: agribotv3 — Clean ROS 2 Jazzy Package

**TL;DR:** Fresh package. Proper ROS 2 structure. `gz.msgs` only. Jazzy-native Nav2 config. EKF fuses wheel odometry + IMU for localization (no AMCL, no GPS, no ground-truth hacks). nicegui web dashboard (zero Qt/venvs). Drop all legacy/duplicate code.

### Localization Strategy
- **EKF** (`robot_localization`) fuses:
  - `/odometry` (wheel odom from Gazebo DiffDrive) → velocities + position
  - `/imu` (Gazebo IMU) → orientation + angular velocity + linear acceleration
- EKF publishes `odom` → `base_link` TF and `/odometry/filtered`
- Nav2 uses `/odometry/filtered` as its odom source
- **No AMCL** — we publish a static `map` → `odom` TF (identity) since we have no LIDAR-based map matching
- **Dropped nodes:** `ground_truth_odom`, `odom_patch`, `odom_to_tf` — EKF handles all of this properly now

### New Structure
```
src/agribotv3/
├── package.xml
├── setup.py
├── resource/agribotv3
├── agribotv3/             # Python module
│   ├── __init__.py
│   ├── teleop.py
│   ├── ultrasonic_converter.py
│   ├── vision_node.py
│   ├── csv_costmap_node.py
│   ├── static_map_tf.py    # Publishes static map→odom identity TF
│   └── gui/                # nicegui web dashboard
│       ├── __init__.py
│       ├── app.py           # entry point (web server)
│       ├── config.py
│       ├── database.py
│       ├── controller.py    # ROS controller (pure threading, no Qt)
│       ├── map_renderer.py  # HTML5 Canvas map
│       └── fast_planner.py
├── config/
│   ├── nav2_params.yaml     # Jazzy-native, no AMCL
│   ├── ekf.yaml             # Single EKF: odom + IMU fusion
│   └── rviz/
│       └── nav.rviz
├── data/                    # CSV seed data
│   ├── grid_cells.csv
│   ├── seeds.csv
│   └── soil_scans.csv
├── launch/
│   ├── sim.launch.py        # Gazebo + bridge + RSP + EKF + static TF
│   ├── nav.launch.py        # Nav2 only (no AMCL, no map_server)
│   └── full.launch.py       # Everything
├── urdf/                    # Keep working URDF as-is
├── meshes/
├── models/
└── worlds/
```

### Key Changes from v2
| Area | v2 | v3 |
|------|----|----|
| GUI | PyQt5 (Qt conflicts, venvs) | nicegui (web, pip install, zero Qt) |
| Bridge msgs | Mixed `ignition.msgs` / `gz.msgs` | `gz.msgs` only (Jazzy) |
| Localization | ground_truth_odom / odom_patch / odom_to_tf hacks | Single EKF (odom + IMU) |
| Nav2 config | Frankensteined Humble→Jazzy, AMCL+map_server | Jazzy-native, no AMCL, static map→odom TF |
| Nav2 launch | `bringup_launch.py` (includes AMCL/map_server) | `navigation_launch.py` only (planner + controller + costmaps) |
| Odom topics | `/odom`, `/odom_raw`, `/odometry`, `/odometry/fixed` | `/odometry` (raw) → EKF → `/odometry/filtered` |
| Venvs | 2 separate venvs | None needed |
| Legacy code | dashboard.py, astar.py, vision.py, odom_patch, odom_to_tf, ground_truth_odom | All dropped |
| Package structure | Flat + GUI subfolder hacks | Proper ament_python |
| Controller | QThread-based | threading.Thread-based |

### Steps (to execute)
1. Scaffold package (`package.xml`, `setup.py`, `resource/`)
2. Migrate kept nodes (teleop, ultrasonic_converter, vision_node, csv_costmap_node)
3. Create `static_map_tf.py` (publishes map→odom static TF)
4. Write `ekf.yaml` — single EKF fusing `/odometry` + `/imu`
5. Write `nav2_params.yaml` from Jazzy defaults (no AMCL section)
6. Build launch files: `sim.launch.py` (Gazebo + bridge + RSP + EKF + static TF), `nav.launch.py` (Nav2 `navigation_launch.py` only), `full.launch.py`
7. Copy URDF/meshes/models/worlds as-is
8. Build nicegui web dashboard (port controller, map, database)
9. Create data/ CSV defaults

### Dropped files (all of these are gone)
- `ground_truth_odom.py` — EKF replaces it
- `odom_patch.py` — EKF replaces it
- `odom_to_tf.py` — EKF replaces it
- `obstacle_avoidance.py` — Nav2 costmap + collision_monitor replaces it
- `dashboard.py` — legacy monolith
- `astar.py` — superseded by `fast_planner.py`
- `vision.py` — superseded by `vision_node.py`
- `ekf_local.yaml`, `ekf_global.yaml`, `navsat_transform.yaml` — single EKF now
- `blank_map.yaml`, `blank_map.pgm` — no map_server needed
- All venv setup logic in launch files

### TF Tree (clean)
```
map (static) → odom (EKF) → base_link
                               ├── Wheel1..4
                               ├── US1..4
                               ├── camera
                               └── Seeder
```

### ROS Topic Map (cleaned up)
| Topic | Type | Direction | Source |
|-------|------|-----------|--------|
| `/cmd_vel` | `Twist` | Pub/Sub | teleop / Nav2 controller / GUI controller |
| `/odometry` | `Odometry` | Bridge (GZ→ROS) | Gazebo DiffDrive |
| `/odometry/filtered` | `Odometry` | Pub | EKF (robot_localization) |
| `/imu` | `Imu` | Bridge (GZ→ROS) | Gazebo IMU |
| `/tf` | `TFMessage` | Bridge + EKF + static | Gazebo joints, EKF odom→base_link, static map→odom |
| `/clock` | `Clock` | Bridge (GZ→ROS) | Gazebo |
| `/joint_states` | `JointState` | Bridge (GZ→ROS) | Gazebo |
| `/ultra/us{1-4}/scan` | `LaserScan` | Bridge (GZ→ROS) | Gazebo sensors |
| `/ultra/us{1-4}` | `Range` | Pub | ultrasonic_converter |
| `/camera/image_raw` | `Image` | Bridge (GZ→ROS) | Gazebo camera |
| `/camera/camera_info` | `CameraInfo` | Bridge (GZ→ROS) | Gazebo camera |
| `/detected_plants` | `PointCloud2` | Pub | csv_costmap_node |
| `/vision/plant_detections` | `String` (JSON) | Pub | vision_node |
| `/vision/optimized_path` | `String` (JSON) | Pub | vision_node |
| `/vision/status` | `String` (JSON) | Pub | vision_node |
| `/vision/robot_pose` | `String` (JSON) | Pub | controller (GUI) |
| `/vision/optimize_path` | `String` (JSON) | Pub | controller (GUI) |
| `/vision/add_obstacle` | `String` (JSON) | Pub | controller (GUI) |
| `/vision/clear_obstacles` | `String` (JSON) | Pub | controller (GUI) |

### Node Summary (v3)
| Node | Entry Point | Key Function |
|------|-------------|--------------|
| `teleop` | `teleop:main` | WASD keyboard teleop → `/cmd_vel` |
| `ultrasonic_converter` | `ultrasonic_converter:main` | LaserScan → Range + spinning TF |
| `vision_node` | `vision_node:main` | Green HSV detect + scipy path opt |
| `csv_costmap_node` | `csv_costmap_node:main` | CSV plants → PointCloud2 for costmap |
| `static_map_tf` | `static_map_tf:main` | Publishes static `map` → `odom` identity TF |
| `gui` | `gui.app:main` | nicegui web dashboard + ROS controller |

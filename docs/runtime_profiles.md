# Runtime profiles

This project supports two odometry source profiles for `robot_localization` EKF.

## Selected profile (SIM)

Pipeline:

`/odom_gt -> EKF -> /odometry/filtered`

- `/odom_gt` is produced by `ground_truth_odom` from Gazebo dynamic poses.
- EKF publishes `/odometry/filtered` and the `odom -> base_link` transform.
- `/imu` is bridged in simulation but intentionally not fused in the SIM profile.

## Future profile (HW)

Pipeline (planned):

`/odometry + /imu -> EKF -> /odometry/filtered`

- `/odometry` should come from wheel odometry on the physical robot.
- `/imu` should come from calibrated onboard IMU data.
- EKF remains the single source of `/odometry/filtered` consumed by Nav2 and GUI.

## Topic contract (mandatory at runtime)

The following topics are required for GUI and Nav2 behavior.

| Topic | Type | Producer | Required by | Notes |
|---|---|---|---|---|
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | `robot_localization/ekf_node` | GUI, Nav2 | Mandatory in both SIM and HW profiles. |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | GUI/teleop/Nav2 controller | Simulator or robot base | Command path for robot motion. |
| `/plan` | `nav_msgs/msg/Path` | Nav2 planner | GUI | GUI path visualization/monitoring. |
| `/vision/plant_detections` | `std_msgs/msg/String` | `vision_node` | GUI | Detection panel/status feed. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Nav2 map server | Nav2, RViz/GUI tools | Global map for planning. |
| `/tf` and `/tf_static` | `tf2_msgs/msg/TFMessage` | RSP, EKF, static TFs | Nav2, RViz, GUI tools | Must include `map`, `odom`, `base_link`, sensor frames. |
| `/ultra/us1/scan_fixed` .. `/ultra/us4/scan_fixed` | `sensor_msgs/msg/LaserScan` | `ultrasonic_converter` | Nav2 costmaps | Obstacle layers use all four topics. |

## Notes on profile selection

- Current launch defaults are aligned to the **SIM** profile.
- HW profile comments are kept as guidance only; no HW toggle is implemented in launch yet.

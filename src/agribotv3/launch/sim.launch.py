"""
sim.launch.py — Gazebo simulation layer for agribotv3

Starts:
  • Gazebo Harmonic with the farm world
  • ros_gz_bridge  (all gz.msgs.* topics)
  • robot_state_publisher
  • robot_localization EKF  (odom → base_link)
  • static_map_tf            (map → odom)
  • ultrasonic_converter
  • csv_costmap_node
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'agribotv3'
    pkg_share = get_package_share_directory(pkg)

    # Process xacro → URDF string (creates US*_scan_link TF frames)
    xacro_path = os.path.join(pkg_share, 'urdf', 'THEBOT', 'robot.xacro')
    robot_desc = xacro.process_file(xacro_path).toxml()

    install_dir = os.path.dirname(pkg_share)
    worlds_dir = os.path.join(pkg_share, 'worlds')
    models_dir = os.path.join(pkg_share, 'models')
    world_file = os.path.join(worlds_dir, 'real_copy.sdf')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    data_dir = os.path.join(pkg_share, 'data')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_reserved_bridges = LaunchConfiguration('enable_reserved_bridges')

    # ── environment ──────────────────────────────────────────
    env_gz = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{install_dir}:{worlds_dir}:{models_dir}',
    )
    env_gpu1 = SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1')
    env_gpu2 = SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia')

    # Strip Snap GTK path to avoid Gazebo crash
    if 'GTK_PATH' in os.environ:
        os.environ.pop('GTK_PATH')
    if 'GDK_PIXBUF_MODULE_FILE' in os.environ:
        os.environ.pop('GDK_PIXBUF_MODULE_FILE')

    # ── Gazebo ────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'AgriBot', '-z', '0.5'],
        output='screen',
    )

    # ── Robot state publisher ─────────────────────────────────
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}],
    )

    # ── ros_gz_bridge ─────────────────────────────────────────
    # Active bridges used by today's simulation stack.
    # Keep this list lean: control, odom/odom_gt source, ultras, camera, clock, TF plumbing.
    active_bridge_arguments = [
        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        # Bridge dynamic_pose as TFMessage to a PRIVATE topic
        # (NOT /tf — that would conflict with RSP).
        # ground_truth_odom node extracts model pose → /odom_gt.
        '/world/SinaiAgri/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/ultra/us1@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/ultra/us2@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/ultra/us3@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/ultra/us4@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    ]

    # Reserved bridges for future hardware/testing migration hooks.
    # Examples to keep here as we grow:
    #   - GPS fixes/velocities (NavSatFix/TwistWithCovariance)
    #   - additional IMUs or depth/LiDAR feeds
    #   - optional debug/telemetry sensor topics
    reserved_bridge_arguments = [
        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
    ]

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=active_bridge_arguments,
        remappings=[
            ('/ultra/us1', '/ultra/us1/scan'),
            ('/ultra/us2', '/ultra/us2/scan'),
            ('/ultra/us3', '/ultra/us3/scan'),
            ('/ultra/us4', '/ultra/us4/scan'),
            ('/world/SinaiAgri/dynamic_pose/info', '/gz_dynamic_poses'),
        ],
        output='screen',
    )

    reserved_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=reserved_bridge_arguments,
        condition=IfCondition(enable_reserved_bridges),
        output='screen',
    )

    # ── EKF (robot_localization) ──────────────────────────────
    ekf = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    # ── Static map → odom TF ─────────────────────────────────
    static_map_tf = Node(
        package=pkg, executable='static_map_tf',
        name='static_map_tf', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Ultrasonic converter ──────────────────────────────────
    ultrasonic = Node(
        package=pkg, executable='ultrasonic_converter',
        name='ultrasonic_converter', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── CSV costmap publisher ─────────────────────────────────
    csv_costmap = Node(
        package=pkg, executable='csv_costmap_node',
        name='csv_costmap_node', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'csv_path': os.path.join(data_dir, 'grid_cells.csv'),
        }],
    )

    # ── Ground truth odom (Gazebo dynamic_pose → /odom_gt) ────
    ground_truth = Node(
        package=pkg, executable='ground_truth_odom',
        name='ground_truth_odom', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'enable_reserved_bridges',
            default_value='false',
            description='Enable reserved bridge topics for future hardware/testing feeds.',
        ),
        env_gz, env_gpu1, env_gpu2,
        gazebo, spawn, rsp, bridge, reserved_bridge,
        ground_truth, ekf, static_map_tf,
        ultrasonic, csv_costmap,
    ])

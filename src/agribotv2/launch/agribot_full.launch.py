"""
AgriBot Launch File

This launch file starts:
1. Gazebo simulation with the AgriBot robot
2. Vision/Path node in venv (for CV2 + scipy optimization, using numpy<2.0 for ROS Jazzy compatibility)
3. GUI node in venv (for PyQt5 stability)

Environment Setup:
- Create two virtual environments:
  $ python3 -m venv --system-site-packages ~/.agribot_venv_vision
  $ source ~/.agribot_venv_vision/bin/activate
  $ pip install "numpy<2.0" opencv-python scipy
  
  $ python3 -m venv --system-site-packages ~/.agribot_venv_gui
  $ source ~/.agribot_venv_gui/bin/activate
  $ pip install "numpy<2.0" PyQt5

  ros2 launch agribotv2 agribot_full.launch.py
"""

import os
import sys

# Prevent Snap/Jazzy GTK library conflicts for Gazebo GUI
if 'GTK_PATH' in os.environ:
    os.environ.pop('GTK_PATH')
if 'GDK_PIXBUF_MODULE_FILE' in os.environ:
    os.environ.pop('GDK_PIXBUF_MODULE_FILE')

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    IncludeLaunchDescription, 
    SetEnvironmentVariable,
    GroupAction,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'agribotv2'
    urdf_file_name = 'AgribotURDF7_withsensors.urdf'

    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Replace package:// URIs with absolute paths
    robot_desc = robot_desc.replace(f'package://{package_name}/', f'file://{pkg_share}/')

    # Paths
    install_dir = os.path.dirname(pkg_share)
    # Paths relative to package share
    worlds_path = os.path.join(pkg_share, 'worlds')
    models_path = os.path.join(pkg_share, 'models')
    gui_path = os.path.join(pkg_share, 'GUI')
    
    # Use the real_copy.sdf world with updated plant positions
    agribot_world = os.path.join(worlds_path, 'real_copy.sdf')

    # Environment variables
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_dir + ':' + worlds_path + ':' + models_path
    )
    set_gpu_offload = SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1')
    set_glx_vendor = SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia')

    # ============ GAZEBO SIMULATION ============
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {agribot_world}'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'AgriBot', '-z', '0.5'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        arguments=[urdf_path]
    )

    # ============ ROS-GAZEBO BRIDGE ============
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/world/SinaiAgri/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/ultra/us1@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ultra/us2@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ultra/us3@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ultra/us4@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat'
        ],
        output='screen',
        remappings=[
            ('/ultra/us1', '/ultra/us1/scan'),
            ('/ultra/us2', '/ultra/us2/scan'),
            ('/ultra/us3', '/ultra/us3/scan'),
            ('/ultra/us4', '/ultra/us4/scan'),
        ]
    )

    # Ultrasonic converter
    ultrasonic_converter = Node(
        package=package_name,
        executable='ultrasonic_converter',
        name='ultrasonic_converter',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # CSV to Costmap for Dynamic Plants
    csv_to_costmap = Node(
        package=package_name,
        executable='csv_to_costmap',
        name='csv_to_costmap',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ============ VISION NODE (ROS Jazzy compatible environment) ============
    # This runs in a separate Python environment with numpy<2.0, scipy, opencv
    # Ubuntu 24.04 / Jazzy standardizes generally on numpy < 2 or handles numpy 2 via apt differently.
    # We force <2.0 to avoid breaking ros messages or custom modules.
    vision_venv = os.path.expanduser('~/.agribot_venv_vision')
    vision_python = os.path.join(vision_venv, 'bin', 'python3')
    
    # Check if vision venv exists, otherwise use system python
    if not os.path.exists(vision_python):
        vision_python = 'python3'
        print(f"[WARN] Vision venv not found at {vision_venv}. Using system Python.")
    
    vision_node = ExecuteProcess(
        cmd=[
            '/bin/bash', '-c',
            f'''
            # Source ROS 2 environment
            source /opt/ros/${{ROS_DISTRO}}/setup.bash
            if [ -f "install/setup.bash" ]; then source install/setup.bash; fi
            
            # Check if venv exists, create if not (with system site packages for ROS)
            if [ ! -d "{vision_venv}" ]; then
                echo "[INFO] Creating vision venv at {vision_venv}..."
                python3 -m venv --system-site-packages {vision_venv}
                {vision_venv}/bin/pip install --upgrade pip
                {vision_venv}/bin/pip install "numpy<2.0" scipy opencv-python
            fi
            
            # Run vision node using the module path
            {vision_venv}/bin/python3 -c "
import sys
import os
from ament_index_python.packages import get_package_share_directory
try:
    from agribotv2.vision_node import main
    main()
except ImportError:
    # Fallback for unconventional installations
    pkg_share = get_package_share_directory('agribotv2')
    sys.path.insert(0, pkg_share)
    from agribotv2.vision_node import main
    main()
"
            '''
        ],
        output='screen'
    )

    # ============ GUI NODE (numpy1 environment) ============
    # This runs the PyQt5 GUI with numpy<2.0
    gui_venv = os.path.expanduser('~/.agribot_venv_gui')
    gui_python = os.path.join(gui_venv, 'bin', 'python3')
    
    gui_node = ExecuteProcess(
        cmd=[
            '/bin/bash', '-c',
            f'''
            # Source ROS 2 environment
            source /opt/ros/${{ROS_DISTRO}}/setup.bash
            if [ -f "install/setup.bash" ]; then source install/setup.bash; fi
            
            # Unset GTK_PATH to avoid conflicts
            unset GTK_PATH
            
            # Use venv if it exists
            if [ -d "{gui_venv}" ]; then
                source {gui_venv}/bin/activate
            fi
            
            # Run GUI
            cd {gui_path}
            python3 main.py
            '''
        ],
        output='screen'
    )

    # ============ RVIZ (optional) ============
    rviz_config_file = os.path.join(pkg_share, 'config', 'basic_rvizconfig.rviz')
    rviz = ExecuteProcess(
        cmd=[
            '/bin/bash', '-c', 
            f'unset GTK_PATH; ros2 run rviz2 rviz2 -d {rviz_config_file} --ros-args -p use_sim_time:=true --log-level error'
        ],
        output='screen'
    )

    # ============ NAV2 BRINGUP ============
    nav2_bringup = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'nav2.launch.py')
                ),
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ]
    )

    # ============ LAUNCH DESCRIPTION ============
    return LaunchDescription([
        # Environment
        set_gz_resource_path,
        set_gpu_offload,
        set_glx_vendor,
        
        # Gazebo simulation
        gazebo,
        spawn_entity,
        robot_state_publisher,
        bridge,
        ultrasonic_converter,
        csv_to_costmap,
        
        # Vision node (delayed start to wait for simulation)
        TimerAction(
            period=5.0,
            actions=[vision_node]
        ),
        
        # GUI node (delayed start to wait for everything else)
        TimerAction(
            period=7.0,
            actions=[gui_node]
        ),
        
        # RViz (optional, delayed)
        TimerAction(
            period=10.0,
            actions=[rviz]
        ),
        
        # Nav2 Stack
        nav2_bringup,
    ])

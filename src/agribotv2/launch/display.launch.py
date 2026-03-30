import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    """
    Ground Truth Launch Configuration
    
    This launch file uses Gazebo's perfect simulation data for localization.
    No robot_localization EKF is used - Gazebo directly provides the TF tree.
    
    TF Tree: odom -> base_link (published by Gazebo DiffDrive plugin)
    Odometry: /odom (ground truth from Gazebo)
    
    Use this configuration for:
    - Navigation stack development/testing
    - Baseline performance measurements
    - Debugging without localization uncertainty
    """
    
    package_name = 'agribotv2'
    
    # ========================================================================
    # ROBOT DESCRIPTION
    # ========================================================================
    xacro_file_path = os.path.join(pkg_share, 'urdf', 'THEBOT', 'robot.xacro')
    
    if not os.path.exists(xacro_file_path):
        print(f"\n\nCRITICAL ERROR: Could not find xacro file at: {xacro_file_path}\n\n")

    robot_description_config = Command(['xacro ', xacro_file_path])

    # ========================================================================
    # ENVIRONMENT SETUP
    # ========================================================================
    pkg_share = get_package_share_directory(package_name)
    source_models_path = os.path.join(pkg_share, 'models')
    install_dir = os.path.dirname(pkg_share)
    
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_dir + ':' + source_models_path
    )
    set_gpu_offload = SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1')
    set_glx_vendor = SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia')

    # ========================================================================
    # GAZEBO SIMULATION
    # ========================================================================
    world_file = os.path.join(pkg_share, 'worlds', 'real_copy.sdf')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_file} --render-engine ogre'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'AgriBot', 
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '0.5'
        ],
        output='screen'
    )

    # ========================================================================
    # ROBOT STATE PUBLISHER
    # ========================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_config, value_type=str),
            'use_sim_time': True
        }],
    )

    # ========================================================================
    # ROS-GAZEBO BRIDGE (Ground Truth)
    # ========================================================================
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            
            # World Pose Info (Fortress: dynamic poses for all models)
            '/world/SinaiAgri/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            
            # Control
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            
            # Ultrasonic Sensors
            '/ultra/us1@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/ultra/us2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/ultra/us3@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/ultra/us4@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            
            # Camera
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            
            # IMU & GPS (available but not used in ground truth mode)
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/gps/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat'
        ],
        output='screen',
        remappings=[
            # Remap ultrasonic scans for converter
            ('/ultra/us1', '/ultra/us1/scan'),
            ('/ultra/us2', '/ultra/us2/scan'),
            ('/ultra/us3', '/ultra/us3/scan'),
            ('/ultra/us4', '/ultra/us4/scan'),
        ]
    )

    # ========================================================================
    # SENSOR PROCESSING & TF PUBLISHING
    # ========================================================================
    
    # Ground Truth Odometry (reads Gazebo model pose, publishes as /odom with TF)
    ground_truth_odom = Node(
        package='agribotv2',
        executable='ground_truth_odom',
        name='ground_truth_odom',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Camera TF Patch (fixes Gazebo scoped frame naming)
    tf_camera_patch = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera', 'AgriBot/base_link/camera'],
        output='screen', 
        parameters=[{'use_sim_time': True}]
    )

    # Ultrasonic Converter (LaserScan -> Range messages)
    ultrasonic_converter = Node(
        package='agribotv2', 
        executable='ultrasonic_converter',
        name='ultrasonic_converter', 
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ========================================================================
    # VISUALIZATION
    # ========================================================================
    rviz_config_file = os.path.join(pkg_share, 'config', 'basic_rvizconfig.rviz')
    rviz = ExecuteProcess(
        cmd=[
            '/bin/bash', '-c', 
            f'unset GTK_PATH; ros2 run rviz2 rviz2 -d {rviz_config_file} --ros-args -p use_sim_time:=true'
        ],
        output='screen'
    )

    # ========================================================================
    # AGRIBOT GUI
    # ========================================================================
    gui_path = os.path.join(pkg_share, 'GUI', 'main.py')
    agribot_gui = ExecuteProcess(
        cmd=[
            '/bin/bash', '-c',
            f'cd {os.path.join(pkg_share, "GUI")} && python3 main.py'
        ],
        output='screen'
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    return LaunchDescription([
        # Environment
        set_gz_resource_path, 
        set_gpu_offload, 
        set_glx_vendor,
        
        # Simulation
        gazebo,
        ground_truth_odom,
        spawn_entity, 
        
        # Robot Description
        robot_state_publisher, 
        
        # Bridge (Ground Truth)
        bridge,
        
        # Sensor Processing
        tf_camera_patch, 
        ultrasonic_converter, 
        
        # Visualization
        rviz,
        
        # AgriBot GUI
        agribot_gui
    ])
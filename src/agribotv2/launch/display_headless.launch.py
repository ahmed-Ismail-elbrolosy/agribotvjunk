import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'agribotv2'
    urdf_file_name = 'AgribotURDF7_withsensors.urdf'

    # --- ARGS ---
    # Allow toggling RViz on/off (Default On, because you need to check TFs)
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to launch RViz'
    )
    use_rviz = LaunchConfiguration('use_rviz')

    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    robot_desc = robot_desc.replace(f'package://{package_name}/', f'file://{pkg_share}/')

    # Paths relative to package share
    install_dir = os.path.dirname(pkg_share)
    worlds_path = os.path.join(pkg_share, 'worlds')

    set_gz_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=install_dir + ':' + worlds_path)
    set_gpu_offload = SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1')
    set_glx_vendor = SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia')

    # --- GAZEBO HEADLESS SETUP ---
    world_file = os.path.join(worlds_path, 'emptysurroundedWithBoxes.sdf')
    
    # -s : Server only (Headless - No Window)
    # -r : Run immediately
    # -v 4 : Verbose logging (Useful since we have no GUI to see errors)
    gz_args = f'-r -s -v 4 {world_file}'
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # Spawn Entity
    spawn_entity = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'AgriBot', '-x', '0.0', '-y', '0.0', '-z', '0.5'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    # Bridge (Gazebo -> ROS)
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # Sensors
            '/ultra/us1@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/ultra/us2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/ultra/us3@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/ultra/us4@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/gps/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat'
        ],
        output='screen',
        remappings=[('/odometry', '/odom_raw')]
    )

    # 1. Local EKF (Odom -> Base Link)
    ekf_local_config_path = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    ekf_local_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node_local', output='screen',
        parameters=[ekf_local_config_path, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    # 2. NavSat Transform (GPS -> Odom coords)
    navsat_config_path = os.path.join(pkg_share, 'config', 'navsat_transform.yaml')
    navsat_transform_node = Node(
        package='robot_localization', executable='navsat_transform_node',
        name='navsat_transform_node', output='screen',
        parameters=[navsat_config_path, {'use_sim_time': True}],
        remappings=[
            ('imu/data', 'imu'),
            ('gps/fix', 'gps/fix'),
            ('odometry/filtered', 'odometry/local'),
            ('odometry/gps', 'odometry/gps')
        ]
    )

    # 3. Global EKF (Map -> Odom)
    ekf_global_config_path = os.path.join(pkg_share, 'config', 'ekf_global.yaml')
    ekf_global_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node_global', output='screen',
        parameters=[ekf_global_config_path, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    # Camera Patch
    tf_camera_patch = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera', 'AgriBot/base_link/camera'],
        output='screen', parameters=[{'use_sim_time': True}]
    )

    # Ultrasonic Converter
    ultrasonic_converter = Node(
        package='agribotv2', executable='ultrasonic_converter',
        name='ultrasonic_converter', output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # RViz (Conditional)
    rviz_config_file = os.path.join(pkg_share, 'config', 'basic_rvizconfig.rviz')
    rviz = ExecuteProcess(
        condition=IfCondition(use_rviz),
        cmd=['/bin/bash', '-c', f'unset GTK_PATH; ros2 run rviz2 rviz2 -d {rviz_config_file} --ros-args -p use_sim_time:=true'],
        output='screen'
    )

    return LaunchDescription([
            use_rviz_arg, # Register the arg
            set_gz_resource_path, set_gpu_offload, set_glx_vendor,
            gazebo, spawn_entity, robot_state_publisher, bridge,
            ekf_local_node, 
            navsat_transform_node,
            ekf_global_node,
            tf_camera_patch, ultrasonic_converter, rviz
    ])
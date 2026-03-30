import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'agribotv2'
    urdf_file_name = 'AgribotURDF7_withsensors.urdf'

    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Paths relative to package share
    worlds_path = os.path.join(pkg_share, 'worlds')
    orchard_world = os.path.join(worlds_path, 'orchard.sdf')
    
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_dir + ':' + worlds_path + ':' + os.path.join(pkg_share, 'models')
    )

    # Force GPU usage for NVIDIA Optimus systems
    set_gpu_offload = SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1')
    set_glx_vendor = SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia')

    # Gazebo Sim launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {orchard_world}'}.items(),
    )

    # Spawn entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'AgriBot',
                   '-z', '0.8'],
        output='screen')

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        arguments=[urdf_path])

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                   '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                   '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                   '/ultra/us1@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                   '/ultra/us2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                   '/ultra/us3@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                   '/ultra/us4@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                   '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                   '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                   '/gps/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat'],
        output='screen'
    )

    # Obstacle Avoidance
    # obstacle_avoidance = Node(
    #     package='agribotv2',
    #     executable='obstacle_avoidance',
    #     name='obstacle_avoidance',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )

    # RViz
    rviz_config_file = os.path.join(pkg_share, 'config', 'urdf_config.rviz')
    # Only unset GTK_PATH, as unsetting LD_LIBRARY_PATH breaks ROS 2 dependencies
    rviz = ExecuteProcess(
        cmd=['/bin/bash', '-c', f'unset GTK_PATH; ros2 run rviz2 rviz2 -d {rviz_config_file} --ros-args -p use_sim_time:=true'],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_gpu_offload,
        set_glx_vendor,
        gazebo,
        spawn_entity,
        robot_state_publisher,
        bridge,
        # obstacle_avoidance,
        rviz,
    ])

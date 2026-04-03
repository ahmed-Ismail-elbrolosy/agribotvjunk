"""
full.launch.py — Everything for agribotv3

Launches sim → Nav2 (delayed) → vision (venv) → GUI (venv)

The .venv lives at <agribotv3_src>/.venv and is created once with:
    cd src/agribotv3
    python3 -m venv --system-site-packages .venv
    .venv/bin/pip install "numpy<2.0" nicegui plotly opencv-python-headless scipy
"""

import os
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Prevent Snap GTK crash before any ROS imports
for _k in ('GTK_PATH', 'GDK_PIXBUF_MODULE_FILE'):
    os.environ.pop(_k, None)


def _find_venv() -> str:
    """Resolve the .venv Python interpreter.

    Search order:
      1. AGRIBOTV3_VENV env var       (explicit override)
      2. Next to this launch file     (symlink-install points here)
      3. <install>/../../../src/agribotv3/.venv
      4. Well-known workspace layout
      5. Fall back to system python3
    """
    # Env override
    env = os.environ.get('AGRIBOTV3_VENV')
    if env:
        p = os.path.join(env, 'bin', 'python3')
        if os.path.isfile(p):
            return p

    # This launch file lives at <src>/agribotv3/launch/ (symlink-install)
    this_dir = os.path.dirname(os.path.realpath(__file__))
    pkg_src = os.path.dirname(this_dir)  # <src>/agribotv3

    pkg_share = get_package_share_directory('agribotv3')

    candidates = [
        os.path.join(pkg_src, '.venv', 'bin', 'python3'),
        # Walk from install/…/share/agribotv3 → workspace/src/agribotv3
        os.path.join(pkg_share, '..', '..', '..', '..', 'src', 'agribotv3', '.venv', 'bin', 'python3'),
        os.path.join(pkg_share, '..', '..', '..', 'src', 'agribotv3', '.venv', 'bin', 'python3'),
    ]
    for c in candidates:
        abspath = os.path.abspath(c)
        if os.path.isfile(abspath):
            return abspath

    # Last resort
    return 'python3'


def generate_launch_description():
    pkg = 'agribotv3'
    pkg_share = get_package_share_directory(pkg)
    launch_dir = os.path.join(pkg_share, 'launch')
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization_mode = LaunchConfiguration('localization_mode')

    venv_python = _find_venv()

    # ── Simulation layer ──────────────────────────────────────
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization_mode': localization_mode,
        }.items(),
    )

    # ── Nav2 (delayed 12 s to let Gazebo + EKF settle) ───────
    nav = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            )
        ],
    )

    # ── Vision node (delayed 5 s, runs inside .venv) ─────────
    vision = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    venv_python, '-c',
                    'from agribotv3.vision_node import main; main()',
                ],
                output='screen',
            )
        ],
    )

    # ── GUI node (delayed 7 s, runs inside .venv) ────────────
    gui = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    venv_python, '-c',
                    'from agribotv3.gui.app import main; main()',
                ],
                output='screen',
            )
        ],
    )

    # ── RViz (delayed 10 s, optional) ────────────────────────
    rviz_cfg = os.path.join(pkg_share, 'config', 'rviz', 'basic_rvizconfig.rviz')
    rviz = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    '/bin/bash', '-c',
                    f'unset GTK_PATH; ros2 run rviz2 rviz2 -d {rviz_cfg} '
                    '--ros-args -p use_sim_time:=true --log-level error',
                ],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'localization_mode',
            default_value='sim_gt',
            choices=['sim_gt', 'hw'],
            description='EKF mode forwarded to sim.launch.py',
        ),
        sim,
        nav,
        vision,
        gui,
        rviz,
    ])

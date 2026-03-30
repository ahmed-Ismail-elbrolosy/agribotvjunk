import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'agribotv3'

setup(
    name=package_name,
    version='3.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'rviz'), glob('config/rviz/*.rviz')),
        # Data (CSV defaults)
        (os.path.join('share', package_name, 'data'), glob('data/*.csv')),
        # URDF
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf') + glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf', 'THEBOT'), glob('urdf/THEBOT/*.xacro')),
        # Meshes
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),
        # Worlds
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf') + glob('worlds/*.world')),
        # Models
        (os.path.join('share', package_name, 'models', 'big_plant'), glob('models/big_plant/*.sdf') + glob('models/big_plant/*.config')),
        (os.path.join('share', package_name, 'models', 'big_plant', 'mesh'), glob('models/big_plant/mesh/*')),
        (os.path.join('share', package_name, 'models', 'small_plant'), glob('models/small_plant/*.sdf') + glob('models/small_plant/*.config')),
        (os.path.join('share', package_name, 'models', 'small_plant', 'mesh'), glob('models/small_plant/mesh/*')),
        (os.path.join('share', package_name, 'models', 'sand_ground'), glob('models/sand_ground/*.sdf') + glob('models/sand_ground/*.config')),
        (os.path.join('share', package_name, 'models', 'sand_ground', 'textures'), glob('models/sand_ground/textures/*')),
        (os.path.join('share', package_name, 'models', 'sand_ground', 'scripts'), glob('models/sand_ground/scripts/*')),
    ],
    install_requires=[
        'setuptools',
        'nicegui',
    ],
    zip_safe=True,
    maintainer='somaa',
    maintainer_email='somaa@todo.todo',
    description='AgriBot v3 — Clean ROS 2 Jazzy navigation package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = agribotv3.teleop:main',
            'ultrasonic_converter = agribotv3.ultrasonic_converter:main',
            'vision_node = agribotv3.vision_node:main',
            'csv_costmap_node = agribotv3.csv_costmap_node:main',
            'static_map_tf = agribotv3.static_map_tf:main',
            'ground_truth_odom = agribotv3.ground_truth_odom:main',
            'gui = agribotv3.gui.app:main',
        ],
    },
)

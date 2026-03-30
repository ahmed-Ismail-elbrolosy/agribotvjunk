import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'agribotv2'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'agribotv2'), glob('agribotv2/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf') + glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf/THEBOT'), glob('urdf/THEBOT/*.xacro')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf') + glob('worlds/*.world')),
        (os.path.join('share', package_name, 'GUI'), glob('GUI/*.py') + glob('GUI/*.jpg') + glob('GUI/*.db') + glob('GUI/*.md')),
        (os.path.join('share', package_name, 'GUI/agribot_data'), glob('GUI/agribot_data/*')),
        (os.path.join('share', package_name, 'models/big_plant'), glob('models/big_plant/*.sdf') + glob('models/big_plant/*.config')),
        (os.path.join('share', package_name, 'models/big_plant/mesh'), glob('models/big_plant/mesh/*')),
        (os.path.join('share', package_name, 'models/small_plant'), glob('models/small_plant/*.sdf') + glob('models/small_plant/*.config')),
        (os.path.join('share', package_name, 'models/small_plant/mesh'), glob('models/small_plant/mesh/*')),
        (os.path.join('share', package_name, 'models/sand_ground'), glob('models/sand_ground/*.sdf') + glob('models/sand_ground/*.config')),
        (os.path.join('share', package_name, 'models/sand_ground/textures'), glob('models/sand_ground/textures/*')),
        (os.path.join('share', package_name, 'models/sand_ground/scripts'), glob('models/sand_ground/scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='somaa',
    maintainer_email='somaa@todo.todo',
    description='Consolidated AgriBot package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = agribotv2.teleop:main',
            'obstacle_avoidance = agribotv2.obstacle_avoidance:main',
            'ultrasonic_converter = agribotv2.ultrasonic_converter:main',
            'odom_patch = agribotv2.odom_patch:main',
            'odom_to_tf = agribotv2.odom_to_tf:main',
            'ground_truth_odom = agribotv2.ground_truth_odom:main',
            'vision_node = agribotv2.vision_node:main',
            'csv_to_costmap = agribotv2.csv_to_costmap:main',
        ],
    },
)

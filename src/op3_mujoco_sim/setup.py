from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'op3_mujoco_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.xml')),
        (os.path.join('share', package_name, 'models', 'assets'), glob('models/assets/*.stl')),
        (os.path.join('share', package_name, 'models', 'assets', 'merged'), glob('models/assets/merged/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joshua',
    maintainer_email='user@todo.todo',
    description='Simulasi MuJoCo Robot Humanoid Hiro untuk Walking Engine (ROS 2 Jazzy)',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'sim_node = op3_mujoco_sim.mujoco_sim_node:main',
            'walk_teleop = op3_mujoco_sim.walk_teleop:main',
        ],
    },
)

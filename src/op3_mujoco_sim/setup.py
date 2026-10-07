from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'op3_mujoco_sim'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

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
        (os.path.join('share', package_name, 'models', 'assets', 'simplified_convex'), glob('models/assets/simplified_convex/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joshua',
    maintainer_email='user@todo.todo',
    description='Simulasi ROBOTIS OP3 Humanoid menggunakan MuJoCo pada ROS 2 Jazzy',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'sim_node = op3_mujoco_sim.mujoco_sim_node:main',
            'teleop_controller = op3_mujoco_sim.teleop_controller:main',
            'motion_player = op3_mujoco_sim.motion_player:main',
        ],
        
    },
)

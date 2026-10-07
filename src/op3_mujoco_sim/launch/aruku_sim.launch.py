import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('op3_mujoco_sim')
    default_config_path = os.path.join(pkg_share, 'config', 'sim_params.yaml')
    default_model_path = os.path.join(pkg_share, 'models', 'scene.xml')
    default_aruku_config = os.path.abspath('config/')

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Buka window 3D GUI MuJoCo Passive Viewer'
    )

    aruku_config_arg = DeclareLaunchArgument(
        'aruku_config_path',
        default_value=default_aruku_config,
        description='Path ke direktori konfigurasi Aruku (berisi kinematic.json & walking.json)'
    )

    sim_node = Node(
        package='op3_mujoco_sim',
        executable='sim_node',
        name='op3_mujoco_sim_node',
        output='screen',
        parameters=[
            default_config_path,
            {
                'use_gui': LaunchConfiguration('use_gui'),
                'model_path': default_model_path,
            }
        ]
    )

    aruku_node = Node(
        package='aruku',
        executable='main',
        name='aruku_node',
        output='screen',
        arguments=[LaunchConfiguration('aruku_config_path')],
    )

    return LaunchDescription([
        use_gui_arg,
        aruku_config_arg,
        sim_node,
        aruku_node,
    ])

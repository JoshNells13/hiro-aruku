import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim_share = get_package_share_directory('op3_mujoco_sim')
    pkg_aruku_share = get_package_share_directory('aruku')

    default_model_path = os.path.join(pkg_sim_share, 'models', 'scene.xml')
    default_config_path = os.path.join(pkg_aruku_share, 'config')

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Buka window 3D GUI MuJoCo Viewer'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model_path,
        description='Path ke file XML scene MuJoCo'
    )

    aruku_config_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Direktori konfigurasi walking (berisi walking.json & kinematic.json)'
    )

    # 1. Node Simulasi MuJoCo
    sim_node = Node(
        package='op3_mujoco_sim',
        executable='sim_node',
        name='mujoco_sim_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'use_gui': LaunchConfiguration('use_gui'),
            'imu_rate': 100.0,
        }],
    )

    # 2. Node Walking Engine (Aruku)
    # aruku_node = Node(
    #     package='aruku',
    #     executable='main',
    #     name='aruku_node',
    #     output='screen',
    #     arguments=[LaunchConfiguration('config_path')],
    # )

    return LaunchDescription([
        use_gui_arg,
        model_path_arg,
        aruku_config_arg,
        sim_node,
        # aruku_node,
    ])

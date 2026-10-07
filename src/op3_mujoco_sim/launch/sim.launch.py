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

    # Argumen Launch
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Buka window 3D GUI MuJoCo Passive Viewer'
    )

    physics_rate_arg = DeclareLaunchArgument(
        'physics_rate',
        default_value='500.0',
        description='Frekuensi loop fisika MuJoCo dalam Hz'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Frekuensi publikasi topik ROS 2 (/joint_states, /imu) dalam Hz'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model_path,
        description='Path absolut ke file XML scene MuJoCo'
    )

    # Node Simulasi MuJoCo
    sim_node = Node(
        package='op3_mujoco_sim',
        executable='sim_node',
        name='op3_mujoco_sim_node',
        output='screen',
        parameters=[
            default_config_path,
            {
                'use_gui': LaunchConfiguration('use_gui'),
                'physics_rate': LaunchConfiguration('physics_rate'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'model_path': LaunchConfiguration('model_path'),
            }
        ]
    )

    return LaunchDescription([
        use_gui_arg,
        physics_rate_arg,
        publish_rate_arg,
        model_path_arg,
        sim_node
    ])

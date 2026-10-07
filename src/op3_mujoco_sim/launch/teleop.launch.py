from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    teleop_node = Node(
        package='op3_mujoco_sim',
        executable='teleop_controller',
        name='op3_teleop_controller',
        output='screen',
        prefix='xterm -e' if False else '',  # Berjalan langsung di terminal pemanggil
        emulate_tty=True
    )

    return LaunchDescription([
        teleop_node
    ])

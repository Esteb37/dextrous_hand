from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dextrous_hand',
            executable='hand_node',
            name='hand_node',
            parameters=[
                {"is_simulation": True}
            ],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='dextrous_hand',
            executable='teleop_node',
            name='teleop_node',
            emulate_tty=True,
            parameters=[
                {"is_simulation": True}
            ],
        ),
        Node(
            package='dextrous_hand',
            executable='mujoco_node',
            name='mujoco_node',
            emulate_tty=True
        ),
    ])
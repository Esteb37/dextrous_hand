from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dextrous_hand',
            executable='hand_node',
            name='hand_node',
            parameters=[
                {"is_simulation": False},
                {"manual_control": True}
            ],
            emulate_tty=True,
            output='screen'
        ),
    ])
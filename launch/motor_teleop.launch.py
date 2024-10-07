from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dextrous_hand',
            executable='hand_node',
            name='hand_node',
        ),
        Node(
            package='dextrous_hand',
            executable='teleop_node',
            name='teleop_node',
            output='screen'
        ),
    ])
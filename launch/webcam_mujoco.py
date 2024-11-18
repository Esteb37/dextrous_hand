import os
from launch import LaunchDescription
from launch_ros.actions import Node

from dextrous_hand.utils.utils import parent_dir


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="dextrous_hand",
                executable="webcam_mano_node",
                name="webcam_mano_node",
            ),

            # RETARGET NODE
            Node(
                package="dextrous_hand",
                executable="retargeter_node",
                name="retargeter_node",
                parameters=[
                    {
                        "retarget/mjcf_filepath": os.path.join(
                            parent_dir(),
                            "data",
                            "assets",
                            "hand_p4.xml",
                        )
                    },
                    {"retarget/hand_scheme": "p4"},
                ]
            ),

            Node(
                package='dextrous_hand',
                executable='mujoco_node',
                name='mujoco_node',
                emulate_tty=True,
                output='screen',
            ),
        ]
    )

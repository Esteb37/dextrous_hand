import os
from launch_ros.actions import Node
from launch import LaunchDescription

from dextrous_hand.utils.utils import parent_dir, DexNode

def generate_launch_description():
    return LaunchDescription([
        DexNode("mujoco_node"),

        DexNode("retargeter_node",
                output="screen",
                parameters=[
                {
                    "retarget/mjcf_filepath": os.path.join(
                        parent_dir(),
                        "data",
                        "assets",
                        "hh_hand.xml",
                    )
                },
                {"retarget/hand_scheme": "hh"},
            ]
        ),

        Node(package = "rviz2",
             executable="rviz2",
             name="rviz2",
             arguments=["-d",
                        os.path.join(parent_dir(),
                                     "data",
                                     "rviz",
                                     "mano_points.rviz")
                        ],
            ),
    ])

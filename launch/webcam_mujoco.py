import os
from launch_ros.actions import Node
from launch import LaunchDescription

from dextrous_hand.utils.utils import parent_dir, DexNode

def generate_launch_description():
    return LaunchDescription([
        DexNode("webcam_mano_node"),

        DexNode("mujoco_node"),

        DexNode("retargeter_node",
            parameters=[
                {
                    "retarget/mjcf_filepath": os.path.join(
                        parent_dir(),
                        "data",
                        "assets",
                        "urdf",
                        "hh_hand.urdf",
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

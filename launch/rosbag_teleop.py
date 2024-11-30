from launch import LaunchDescription
import os
from launch_ros.actions import Node

from dextrous_hand.utils.utils import parent_dir, DexNode

def generate_launch_description():
    return LaunchDescription([
        DexNode("hand_node"),

        DexNode("retargeter_node",
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

        DexNode("visualize_joints_node",
            parameters=[
                {"scheme_path": os.path.join(
                    parent_dir(),
                    "data",
                    "assets",
                    "scheme_hh.yaml",
                )}
            ]
        ),

        DexNode("wrist_controller_node",
                output="screen",
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

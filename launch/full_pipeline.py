from launch import LaunchDescription
import os
from launch_ros.actions import Node

from dextrous_hand.utils.utils import parent_dir, DexNode


cameras = {"front_view": True, "side_view": True, "wrist_view": True}

def generate_launch_description():
    return LaunchDescription([
        DexNode("hand_node"),

        DexNode("wrist_controller_node"),

        DexNode("rokoko_node",
            parameters=[
                {"rokoko_tracker/ip": "0.0.0.0"},
                {"rokoko_tracker/port": 14043},
                {"rokoko_tracker/use_coil": True}
            ],
        ),

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

        DexNode("oakd_node",
            parameters=[
                {"enable_front_camera": cameras["front_view"]},
                {"enable_side_camera": cameras["side_view"]},
                {"enable_wrist_camera": cameras["wrist_view"]},
            ],
    ),

    ])

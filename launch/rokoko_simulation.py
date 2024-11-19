from launch import LaunchDescription
import os

from dextrous_hand.utils.utils import parent_dir, DexNode

def generate_launch_description():
    return LaunchDescription([
        DexNode("mujoco_node"),

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
                        "hh_hand.xml",
                    )
                },
                {"retarget/hand_scheme": "p4"},
            ]
        )
    ])

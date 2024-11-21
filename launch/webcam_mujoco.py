import os
from launch import LaunchDescription

from dextrous_hand.utils.utils import parent_dir, DexNode

def generate_launch_description():
    return LaunchDescription([
        DexNode("webcam_mano_node"),

        DexNode("mujoco_node", output="screen"),

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
    ])

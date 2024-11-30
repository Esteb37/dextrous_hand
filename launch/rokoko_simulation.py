import os
from launch_ros.actions import Node
from launch import LaunchDescription

from dextrous_hand.utils.utils import parent_dir, DexNode

def generate_launch_description():

    urdf = os.path.join(
                    parent_dir(),
                    "data",
                    "assets",
                    "urdf",
                    "hh_hand.urdf",
                )


    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

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
                        "urdf",
                        "hh_hand.urdf",
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

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc,}],
            arguments=[urdf]
        ),

        Node(package = "rviz2",
             executable="rviz2",
             name="rviz2",
             arguments=["-d",
                        os.path.join(parent_dir(),
                                     "data",
                                     "rviz",
                                     "urdf.rviz")
                        ],
            ),
    ])

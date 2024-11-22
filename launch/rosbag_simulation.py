import os
from launch_ros.actions import Node
from launch import LaunchDescription

from dextrous_hand.utils.utils import parent_dir, DexNode

def generate_launch_description():

    p4_urdf = os.path.join(
                    parent_dir(),
                    "data",
                    "assets",
                    "urdf",
                    "hh_hand.urdf",
                )


    with open(p4_urdf, 'r') as infp:
        p4_robot_desc = infp.read()

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

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='hand',
            name='robot_state_publisher',
            parameters=[{'robot_description': p4_robot_desc,}],
            remappings=[('/joint_states', '/hand/joint_states')]
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

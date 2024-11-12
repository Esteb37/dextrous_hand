import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from dextrous_hand.utils.utils import parent_dir

"""
Visualize the robot hand in rviz and show the GUI to adjust the joint angles.
Does not connect to real robot nor start simulation.
"""

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

        Node(
                package="dextrous_hand",
                executable="webcam_mano_node",
                name="webcam_mano_node",
            ),

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
                        "hh_hand.xml",
                    )
                },
                {"retarget/hand_scheme": "p4"},
            ]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc,}],
            arguments=[urdf]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'),

        Node(
            package="dextrous_hand",
            executable="visualize_joints_node",
            name="visualize_joints_node",
            output='log',
            parameters=[
                {"scheme_path": os.path.join(
                            parent_dir(),
                            "data",
                            "assets",
                            "scheme_p4.yaml",
                        )}
            ]
        ),

    ])
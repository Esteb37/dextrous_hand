import os

from launch_ros.actions import Node
from dextrous_hand.utils.utils import parent_dir
from dextrous_hand.utils.constants import GLOBAL_CONSTANTS

class DexNode(Node):
    def __init__(self, name, **kwargs):
        super().__init__(
            package="dextrous_hand",
            executable=name,
            name=name,
            emulate_tty=True,
            **kwargs)

class SimNode(DexNode):
    def __init__(self, name, **kwargs):
        params = kwargs.pop("parameters", [])
        params.append({"is_simulation": True})
        super().__init__(name, parameters=params, **kwargs)

def control_nodes(sim = False):

    NodeClass = DexNode if not sim else SimNode

    return [
        NodeClass("hand_node"),

        # NodeClass("wrist_controller_node"),

        NodeClass("retargeter_node",
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
        )
    ]

def rokoko_node():
    return [DexNode("rokoko_node",
            parameters=[
                {"rokoko_tracker/ip": "0.0.0.0"},
                {"rokoko_tracker/port": 14043},
                {"rokoko_tracker/use_coil": True}
            ],
        )]

def ingress_nodes():

    cameras = GLOBAL_CONSTANTS["CAMERAS"]

    return [
        DexNode("rokoko_node",
            parameters=[
                {"rokoko_tracker/ip": "0.0.0.0"},
                {"rokoko_tracker/port": 14043},
                {"rokoko_tracker/use_coil": True}
            ],
        ),

        DexNode("oakd_node",
            parameters=[
                {"enable_front_camera": cameras["front_view"]},
                {"enable_side_camera": cameras["side_view"]},
                {"enable_wrist_camera": cameras["wrist_view"]},
            ],
        )]

def viz_nodes():
    urdf = os.path.join(
                    parent_dir(),
                    "data",
                    "assets",
                    "urdf",
                    "hh_hand.urdf",
                )


    with open(urdf, 'r') as infp:
        robot_desc = infp.read()


    nodes = [
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
            )
    ]

    if GLOBAL_CONSTANTS["WITH_MUJOCO"]:
        nodes.append(DexNode("mujoco_node"))

    return nodes
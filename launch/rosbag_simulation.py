from launch import LaunchDescription

from dextrous_hand.utils.node_utils import control_nodes, viz_nodes

def generate_launch_description():
    return LaunchDescription(
        control_nodes(sim = True) + viz_nodes()
    )

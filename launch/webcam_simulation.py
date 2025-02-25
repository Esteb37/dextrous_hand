
from launch import LaunchDescription

from dextrous_hand.utils.node_utils import DexNode, control_nodes, viz_nodes

def generate_launch_description():
    return LaunchDescription(
        [DexNode("webcam_mano_node")] + \
         viz_nodes() + \
         control_nodes(sim = True)
    )
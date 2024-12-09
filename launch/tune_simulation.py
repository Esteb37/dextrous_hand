from launch import LaunchDescription

from dextrous_hand.utils.node_utils import control_nodes, viz_nodes, rokoko_node, DexNode

def generate_launch_description():
    return LaunchDescription(
        [DexNode("slider_node")] + \
         viz_nodes() + \
         rokoko_node() + \
         control_nodes(sim = True)
    )

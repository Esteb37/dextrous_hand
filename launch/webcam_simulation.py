
from launch import LaunchDescription

from dextrous_hand.utils.node_utils import DexNode, control_nodes, viz_nodes

with_mujoco = False

def generate_launch_description():
    return LaunchDescription(
        [DexNode("webcam_mano_node")] + \
         control_nodes(sim = True) + \
         viz_nodes(with_mujoco)
    )
from launch import LaunchDescription

from dextrous_hand.utils.node_utils import control_nodes, viz_nodes, ingress_nodes, DexNode

configuration = "dice"
with_mujoco = False

def generate_launch_description():
    return LaunchDescription(
        [DexNode("slider_node")] + \
         control_nodes(configuration) + \
         ingress_nodes() + \
         viz_nodes(with_mujoco)
    )

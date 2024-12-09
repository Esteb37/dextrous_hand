from launch import LaunchDescription
from dextrous_hand.utils.node_utils import control_nodes, ingress_nodes, viz_nodes

def generate_launch_description():
    return LaunchDescription(
        ingress_nodes() + \
        control_nodes() + \
        viz_nodes()
    )

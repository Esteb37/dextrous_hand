from launch import LaunchDescription

from dextrous_hand.utils.node_utils import control_nodes, viz_nodes

configuration = "default"
with_mujoco = False

def generate_launch_description():
    return LaunchDescription(
        control_nodes(configuration) + viz_nodes(with_mujoco)
    )

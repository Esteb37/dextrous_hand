from launch import LaunchDescription

from dextrous_hand.utils.node_utils import control_nodes, ingress_nodes, viz_nodes

configuration = "default"
cameras = {"front_view": False, "side_view": False, "wrist_view": False}
with_mujoco = False

def generate_launch_description():
    return LaunchDescription(
        ingress_nodes(cameras) + \
        control_nodes(configuration, sim = True) + \
        viz_nodes(with_mujoco)
    )

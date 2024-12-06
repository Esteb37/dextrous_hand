from launch import LaunchDescription
from dextrous_hand.utils.node_utils import DexNode, SimNode

def generate_launch_description():
    return LaunchDescription([
        SimNode("hand_node", output="screen"),
        DexNode("mujoco_node"),
        SimNode("keyboard_node"),
    ])
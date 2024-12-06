from launch import LaunchDescription
from dextrous_hand.utils.node_utils import DexNode

def generate_launch_description():
    return LaunchDescription([
        DexNode("hand_node", output="screen"),
        DexNode("mujoco_node"),
        DexNode("keyboard_node"),
    ])
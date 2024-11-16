from launch import LaunchDescription
from dextrous_hand.utils.utils import DexNode

def generate_launch_description():
    return LaunchDescription([
        DexNode("hand_node"),
        DexNode("keyboard_node"),
    ])
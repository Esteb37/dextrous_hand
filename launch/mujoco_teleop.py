from launch import LaunchDescription
from dextrous_hand.utils.utils import DexNode

def generate_launch_description():
    return LaunchDescription([
        DexNode("mujoco_controller_node"),
        DexNode("hand_node", output="screen"),
    ])
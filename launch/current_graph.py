from launch_ros.actions import Node
from launch import LaunchDescription
from dextrous_hand.utils.ids import MOTORS
from dextrous_hand.utils.node_utils import DexNode

def generate_launch_description():

    current_topics = ["/current/"+motor.name+"/data" for motor in MOTORS]

    arguments = current_topics

    return LaunchDescription([
        DexNode("hand_node"),
        DexNode("keyboard_node"),
        Node(
            package="rqt_plot",
            executable="rqt_plot",
            name="rqt_plot",
            arguments=arguments,
        )

    ])
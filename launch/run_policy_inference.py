from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from dextrous_hand.utils.node_utils import DexNode
from dextrous_hand.utils.constants import GLOBAL_CONSTANTS

def generate_launch_description():
    # Declare launch arguments with default values
    big_policy_ckpt_arg = DeclareLaunchArgument(
        'big_policy_ckpt_path',
        default_value=GLOBAL_CONSTANTS["BIG_MODEL_PATH"],
        description='The ckpt path of the model to load. There should be `config.yaml` in its directory or parent directory'
    )

    small_policy_ckpt_arg = DeclareLaunchArgument(
        'small_policy_ckpt_path',
        default_value=GLOBAL_CONSTANTS["SMALL_MODEL_PATH"],
        description='The ckpt path of the model to load. There should be `config.yaml` in its directory or parent directory'
    )

    # Use LaunchConfiguration to capture the values passed via command line
    big_policy_ckpt_path = LaunchConfiguration('big_policy_ckpt_path')
    small_policy_ckpt_path = LaunchConfiguration('small_policy_ckpt_path')

    # Define the node with parameters from the launch arguments
    policy_node = DexNode(
        name="model_inference_node",
        output='screen',
        parameters=[{
            'camera_topics': [
                "/oakd_front_view/color", "/oakd_side_view/color", "/oakd_wrist_view/color"
            ],
            'camera_names': [
                "oakd_front_view_images", "oakd_side_view_images", "oakd_wrist_view_images"
            ],
            'big_policy_ckpt_path': big_policy_ckpt_path,
            'small_policy_ckpt_path': small_policy_ckpt_path,
         }]
    )

    # Return the LaunchDescription with all the launch arguments and nodes
    return LaunchDescription([
        big_policy_ckpt_arg,
        small_policy_ckpt_arg,
        policy_node,
        DexNode("motors_node"),
        DexNode("oakd_node"),
        DexNode("yolo_node"),
])
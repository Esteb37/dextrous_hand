#!/usr/bin/env python3

import yaml
import os

from std_msgs.msg import Float32MultiArray, Float32, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from dextrous_hand.utils.utils import parent_dir

constants_dir = parent_dir() + '/data/constants/'

retargeter_dir = parent_dir() + '/data/retargeter/'

ARM_CONSTANTS = yaml.safe_load(open(constants_dir + 'arm.yaml'))
GLOBAL_CONSTANTS  = yaml.safe_load(open(constants_dir + 'global.yaml'))
MOTOR_CONSTANTS = yaml.safe_load(open(constants_dir + 'motors.yaml'))
CONFIGS = yaml.safe_load(open(constants_dir + 'configs.yaml'))
RETARGETER_PARAMS = {}


# Go through all files in retargeter_dir
files = [f for f in os.listdir(retargeter_dir) if os.path.isfile(os.path.join(retargeter_dir, f))]
for file in files:
    file_contents = yaml.safe_load(open(retargeter_dir + file))
    title = file_contents['title']
    RETARGETER_PARAMS[title] = file_contents

    if title != 'global' and len(file_contents["keyvectors"]) != len(file_contents["loss_coeffs"].values()):
        raise ValueError("In "+title+".yaml, there must be a loss_coeff for each keyvector. The number of keyvectors and loss_coeffs must be the same.")

NODE_FREQUENCY_HZ = GLOBAL_CONSTANTS['NODE_FREQUENCY_HZ']


# Define supported topics and message types
LOGGER_TOPICS_TYPES = {
    # Motors
    "/motors/target/positions": Float32MultiArray,

    "/motors/read/positions": Float32MultiArray,
    "/motors/read/dxl_currents": Float32MultiArray,

    # FRANKA ROBOT
    "/franka/end_effector_pose": PoseStamped,
    "/franka/end_effector_pose_cmd": PoseStamped,

    # CAMERA IMAGES
    "/oakd_front_view/color": Image,
    "/oakd_front_view/depth": Image,
    "/oakd_side_view/color": Image,
    "/oakd_side_view/depth": Image,

    "/oakd_wrist_view/color": Image,

    "/task_description": String,  # New topic for task description

    # CAMERA PARAMETERS
    "/oakd_front_view/intrinsics": Float32MultiArray,
    "/oakd_side_view/intrinsics": Float32MultiArray,
    "/oakd_wrist_view/intrinsics": Float32MultiArray,

    "/oakd_front_view/extrinsics": Float32MultiArray,
    "/oakd_side_view/extrinsics": Float32MultiArray,

    "/oakd_front_view/projection": Float32MultiArray,
    "/oakd_side_view/projection": Float32MultiArray,

    # Rokoko data
    "/ingress/mano": Float32MultiArray,
    "/ingress/wrist": PoseStamped,
}

IMAGE_TOPIC_TYPES = {
    "/observations/images/oakd_front_view/color": Image,
    "/observations/images/oakd_side_view/color": Image,
}
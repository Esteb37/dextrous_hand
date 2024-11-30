#!/usr/bin/env python3

import yaml

from std_msgs.msg import Float32MultiArray, Float32, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

from dextrous_hand.utils.utils import parent_dir

constants_dir = parent_dir() + '/data/constants/'

ARM_CONSTANTS = yaml.safe_load(open(constants_dir + 'arm.yaml'))
GLOBAL_CONSTANTS  = yaml.safe_load(open(constants_dir + 'global.yaml'))
MOTOR_CONSTANTS = yaml.safe_load(open(constants_dir + 'motors.yaml'))
CONFIGS = yaml.safe_load(open(constants_dir + 'configs.yaml'))
RETARGETER_PARAMS = yaml.safe_load(open(constants_dir + 'retargeter_params.yaml'))

if len(RETARGETER_PARAMS["keyvectors"]) != len(RETARGETER_PARAMS["loss_coeffs"].values()):
    raise ValueError("In retargeter_params.yaml, there must be a loss_coeff for each keyvector. The number of keyvectors and loss_coeffs must be the same.")

NODE_FREQUENCY_HZ = GLOBAL_CONSTANTS['NODE_FREQUENCY_HZ']


# Define supported topics and message types
LOGGER_TOPICS_TYPES = {
    # HandCnfig
    "/hand_config": Float32MultiArray,
    "/current_hand_config": Float32MultiArray,
    "/wrist_cmd": Float32,

    # Motors
    "/motors/target/positions": Float32MultiArray,
    "/motors/target/dxl_positions" : Float32MultiArray,

    "/motors/read/positions": Float32MultiArray,
    "/motors/read/dxl_positions": Float32MultiArray,
    "/motors/read/dxl_velocities": Float32MultiArray,
    "/motors/read/dxl_currents": Float32MultiArray,

    # FRANKA ROBOT
    "/franka/end_effector_pose": PoseStamped,
    "/franka/end_effector_pose_cmd": PoseStamped,

    # CAMERA IMAGES
    "/oakd_front_view/color": Image,
    "/oakd_side_view/color": Image,
    "/oakd_wrist_view/color": Image,

    "/task_description": String,  # New topic for task description

    # CAMERA PARAMETERS
    "/oakd_front_view/intrinsics": Float32MultiArray,
    "/oakd_side_view/intrinsics": Float32MultiArray,
    "/oakd_wrist_view/intrinsics": Float32MultiArray,
    "/oakd_front_view/extrinsics": Float32MultiArray,
    "/oakd_side_view/extrinsics": Float32MultiArray,
    "/oakd_wrist_view/extrinsics": Float32MultiArray,
    "/oakd_front_view/projection": Float32MultiArray,
    "/oakd_side_view/projection": Float32MultiArray,
    "/oakd_wrist_view/projection": Float32MultiArray,

    # Rokoko data
    "/ingress/mano": Float32MultiArray,
    "/ingress/wrist": PoseStamped,
    "/ingress/elbow": PoseStamped,

}
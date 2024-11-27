#!/usr/bin/env python3

import yaml

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
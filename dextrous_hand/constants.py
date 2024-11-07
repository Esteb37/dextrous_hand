#!/usr/bin/env python3

import yaml
from dextrous_hand.utils import parent_dir
constants_dir = parent_dir() + '/data/constants/'

ARM_CONSTANTS = yaml.safe_load(open(constants_dir + 'arm.yaml'))
GLOBAL_CONSTANTS  = yaml.safe_load(open(constants_dir + 'global.yaml'))
MOTOR_CONSTANTS = yaml.safe_load(open(constants_dir + 'motors.yaml'))
CONFIGS = yaml.safe_load(open(constants_dir + 'configs.yaml'))

IS_SIMULATION = GLOBAL_CONSTANTS['IS_SIMULATION']
MANUAL_CONTROL = GLOBAL_CONSTANTS['MANUAL_CONTROL']
NODE_FREQUENCY_HZ = GLOBAL_CONSTANTS['NODE_FREQUENCY_HZ']
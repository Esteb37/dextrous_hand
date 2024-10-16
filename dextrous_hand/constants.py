#!/usr/bin/env python3

from numpy import pi as PI
from dextrous_hand.ids import MOTORS

IS_SIMULATION = True # Set to True if the motor controller is not connected
DEVICE_NAME = "/dev/ttyUSB0" # Change this if the motor controller is connected to a different port
BAUDRATE = 3000000

FULL_RANGE = [-PI, PI]
MOTOR_LIMITS = {
    MOTORS.THUMB_ULBR: FULL_RANGE,
    MOTORS.THUMB_URBL: FULL_RANGE,
    MOTORS.THUMB_UMBM: FULL_RANGE,

    MOTORS.INDEX_ULBR: FULL_RANGE,
    MOTORS.INDEX_URBL: FULL_RANGE,
    MOTORS.INDEX_UMBM: FULL_RANGE,

    MOTORS.MIDDLE_ULBR: FULL_RANGE,
    MOTORS.MIDDLE_URBL: FULL_RANGE,
    MOTORS.MIDDLE_UMBM: FULL_RANGE,

    MOTORS.RING_ULBR: FULL_RANGE,
    MOTORS.RING_URBL: FULL_RANGE,
    MOTORS.RING_UMBM: FULL_RANGE,

    MOTORS.PINKY_ULBR: FULL_RANGE,
    MOTORS.PINKY_URBL: FULL_RANGE,
    MOTORS.PINKY_UMBM: FULL_RANGE,

    MOTORS.WRIST: FULL_RANGE
}
"""
Hard limits for the motors to prevent overextension.
"""
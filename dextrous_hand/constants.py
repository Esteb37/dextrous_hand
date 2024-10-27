#!/usr/bin/env python3

from numpy import pi as PI
from dextrous_hand.ids import MOTORS

IS_SIMULATION = False # Set to True if the motor controller is not connected
DEVICE_NAME = "/dev/ttyUSB0" # Change this if the motor controller is connected to a different port
BAUDRATE = 3000000

NODE_FREQUENCY_HZ = 1000

SPOOL_RADIUS = 1

FULL_RANGE = [-PI, PI]
MOTOR_LIMITS = {
    MOTORS.THUMB_TLBR: FULL_RANGE,
    MOTORS.THUMB_TRBL: FULL_RANGE,
    MOTORS.THUMB_TMBM: FULL_RANGE,

    MOTORS.INDEX_TLBR: FULL_RANGE,
    MOTORS.INDEX_TRBL: FULL_RANGE,
    MOTORS.INDEX_TMBM: FULL_RANGE,

    MOTORS.MIDDLE_TLBR: FULL_RANGE,
    MOTORS.MIDDLE_TRBL: FULL_RANGE,
    MOTORS.MIDDLE_TMBM: FULL_RANGE,

    MOTORS.RING_TLBR: FULL_RANGE,
    MOTORS.RING_TRBL: FULL_RANGE,
    MOTORS.RING_TMBM: FULL_RANGE,

    MOTORS.PINKY_TLBR: FULL_RANGE,
    MOTORS.PINKY_TRBL: FULL_RANGE,
    MOTORS.PINKY_TMBM: FULL_RANGE,

    MOTORS.WRIST: FULL_RANGE
}
"""
Hard limits for the motors to prevent overextension.
"""
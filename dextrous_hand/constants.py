#!/usr/bin/env python3

from numpy import pi as PI
from dextrous_hand.ids import MOTORS, STARTUP
from dextrous_hand.HandConfig import HandConfig

IS_SIMULATION = False # Set to True if the motor controller is not connected
DEVICE_NAME = "/dev/ttyUSB0" # Change this if the motor controller is connected to a different port
BAUDRATE = 3000000

NODE_FREQUENCY_HZ = 1000

STARTUP_MODE = STARTUP.LAST
"""
Defines the startup mode for the hand. Options are:
- HOME: The hand will start in the home configuration.
- LAST: The hand will start in the last configuration it was in.
- CUSTOM: The hand will start in the configuration defined by the INITIAL_CONFIG variable.
"""

INITIAL_CONFIG = HandConfig(
    PINKY = [0, 0, 0],
    RING = [0, 0, 0],
    MIDDLE = [0, 0, 0],
    INDEX = [0, 0, 0],
    THUMB = [0, 0, 0],
    WRIST = [0]
)
"""
Custom starting configuration
"""

FULL_RANGE = [0, 2*PI]
MOTOR_LIMITS = {
    MOTORS.THUMB_FLBR: FULL_RANGE,
    MOTORS.THUMB_FRBL: FULL_RANGE,
    MOTORS.THUMB_FMBM: FULL_RANGE,

    MOTORS.INDEX_FLBR: FULL_RANGE,
    MOTORS.INDEX_FRBL: FULL_RANGE,
    MOTORS.INDEX_FMBM: FULL_RANGE,

    MOTORS.MIDDLE_FLBR: FULL_RANGE,
    MOTORS.MIDDLE_FRBL: FULL_RANGE,
    MOTORS.MIDDLE_FMBM: FULL_RANGE,

    MOTORS.RING_FLBR: FULL_RANGE,
    MOTORS.RING_FRBL: FULL_RANGE,
    MOTORS.RING_FMBM: FULL_RANGE,

    MOTORS.PINKY_FLBR: FULL_RANGE,
    MOTORS.PINKY_FRBL: FULL_RANGE,
    MOTORS.PINKY_FMBM: FULL_RANGE,

    MOTORS.WRIST: FULL_RANGE
}
"""
Hard limits for the motors to prevent overextension.
"""

MOTOR_ZEROS = {
    MOTORS.THUMB_FLBR: 0,
    MOTORS.THUMB_FRBL: 0,
    MOTORS.THUMB_FMBM: 0,

    MOTORS.INDEX_FLBR: 0,
    MOTORS.INDEX_FRBL: 0,
    MOTORS.INDEX_FMBM: 0,

    MOTORS.MIDDLE_FLBR: 0,
    MOTORS.MIDDLE_FRBL: 0,
    MOTORS.MIDDLE_FMBM: 0,

    MOTORS.RING_FLBR: 0,
    MOTORS.RING_FRBL: 0,
    MOTORS.RING_FMBM: 0,

    MOTORS.PINKY_FLBR: 0,
    MOTORS.PINKY_FRBL: 0,
    MOTORS.PINKY_FMBM: 0,

    MOTORS.WRIST: 0
}
"""
Offset to zero the motors.
"""
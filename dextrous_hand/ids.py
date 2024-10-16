#!/usr/bin/env python3

from enum import Enum, auto
from numpy import pi

class MOTORS(Enum):
    """
    Motor ports
    """
    PINKY_UMBM = 0
    PINKY_URBL = 1
    PINKY_ULBR = 2

    RING_UMBM = 3
    RING_URBL = 4
    RING_ULBR = 5

    MIDDLE_UMBM = 6
    MIDDLE_URBL = 7
    MIDDLE_ULBR = 8

    INDEX_UMBM = 9
    INDEX_URBL = 10
    INDEX_ULBR = 11

    THUMB_UMBM = 12
    THUMB_URBL = 13
    THUMB_ULBR = 14

    WRIST = 15

FULL_RANGE = [-pi, pi]
MOTOR_LIMITS = {
    MOTORS.PINKY_UMBM: FULL_RANGE,
    MOTORS.PINKY_URBL: FULL_RANGE,
    MOTORS.PINKY_ULBR: FULL_RANGE,

    MOTORS.RING_UMBM: FULL_RANGE,
    MOTORS.RING_URBL: FULL_RANGE,
    MOTORS.RING_ULBR: FULL_RANGE,

    MOTORS.MIDDLE_UMBM: FULL_RANGE,
    MOTORS.MIDDLE_URBL: FULL_RANGE,
    MOTORS.MIDDLE_ULBR: FULL_RANGE,

    MOTORS.INDEX_UMBM: FULL_RANGE,
    MOTORS.INDEX_URBL: FULL_RANGE,
    MOTORS.INDEX_ULBR: FULL_RANGE,

    MOTORS.THUMB_UMBM: FULL_RANGE,
    MOTORS.THUMB_URBL: FULL_RANGE,
    MOTORS.THUMB_ULBR: FULL_RANGE,

    MOTORS.WRIST: FULL_RANGE
}
"""
Hard limits for the motors to prevent overextension.
"""

class SUBSYSTEMS(Enum):
    """
    Subsystem IDs.
    Each subsystem has a unique ID, determined automatically by the Enum class.
    A subsystem is a unique element of the robot that can be controlled independently.
    """
    PINKY = auto()
    RING = auto()
    MIDDLE = auto()
    INDEX = auto()
    THUMB = auto()
    WRIST = auto()

class JOINTS(Enum):
    """
    Joint IDs.
    Each joint has a unique ID, determined automatically by the Enum class.
    A joint is a component of a subsystem that has a corresponding angle.
    PIP and DIP angles are always coupled.
    """

    PINKY_ABD = auto()
    PINKY_MCP = auto()
    PINKY_PIP = auto()
    PINKY_DIP = auto()

    RING_ABD = auto()
    RING_MCP = auto()
    RING_PIP = auto()
    RING_DIP = auto()

    MIDDLE_ABD = auto()
    MIDDLE_MCP = auto()
    MIDDLE_PIP = auto()
    MIDDLE_DIP = auto()

    INDEX_ABD = auto()
    INDEX_MCP = auto()
    INDEX_PIP = auto()
    INDEX_DIP = auto()

    THUMB_ABD = auto()
    THUMB_MCP = auto()
    THUMB_PIP = auto()
    THUMB_DIP = auto()

    WRIST = auto()

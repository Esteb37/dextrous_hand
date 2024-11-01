#!/usr/bin/env python3

from enum import Enum, auto

class MOTORS(Enum):
    """
    Motor ports
    """
    THUMB_MCP = 0

    RING_FMBM = 1
    RING_FLBR = 2
    RING_FRBL = 3

    PINKY_FRBL = 4
    PINKY_FLBR = 5
    PINKY_FMBM = 6

    THUMB_ABD = 7

    WRIST = 8

    INDEX_FMBM = 9
    INDEX_FLBR = 10
    INDEX_FRBL = 11

    MIDDLE_FRBL = 12
    MIDDLE_FLBR = 13
    MIDDLE_FMBM = 14

    THUMB_PIP = 15

class SUBSYSTEMS(Enum):
    """
    Subsystem IDs.
    Each subsystem has a unique ID, determined automatically by the Enum class.
    A subsystem is a unique element of the robot that can be controlled independently.
    """
    PINKY = 0
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

    PINKY_ABD = 0
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

class STARTUP(Enum):
    HOME = 0 # all joints set to 0
    LAST = auto() # all joints set to their last known position
    CUSTOM = auto() # all joints set to a custom position


class MOTOR_DIRECTION(Enum):
    """
    Motor directions.
    Each motor has a direction that determines the sign of the angle.
    """
    FORWARD = 1
    REVERSED = -1

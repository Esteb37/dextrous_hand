#!/usr/bin/env python3

from enum import Enum, auto
from dextrous_hand.utils.constants import MOTOR_CONSTANTS

class MOTORS(Enum):
    """
    Motor ports. Obtained from the motors.yaml file
    """
    PINKY_FMBM = MOTOR_CONSTANTS["PINKY_FMBM"]["port"]
    PINKY_FRBL = MOTOR_CONSTANTS["PINKY_FRBL"]["port"]
    PINKY_FLBR = MOTOR_CONSTANTS["PINKY_FLBR"]["port"]

    RING_FMBM = MOTOR_CONSTANTS["RING_FMBM"]["port"]
    RING_FRBL = MOTOR_CONSTANTS["RING_FRBL"]["port"]
    RING_FLBR = MOTOR_CONSTANTS["RING_FLBR"]["port"]

    INDEX_FMBM = MOTOR_CONSTANTS["INDEX_FMBM"]["port"]
    INDEX_FRBL = MOTOR_CONSTANTS["INDEX_FRBL"]["port"]
    INDEX_FLBR = MOTOR_CONSTANTS["INDEX_FLBR"]["port"]

    THUMB_ABD = MOTOR_CONSTANTS["THUMB_ABD"]["port"]
    THUMB_MCP = MOTOR_CONSTANTS["THUMB_MCP"]["port"]
    THUMB_PIP = MOTOR_CONSTANTS["THUMB_PIP"]["port"]

    MIDDLE_FRBL = MOTOR_CONSTANTS["MIDDLE_FRBL"]["port"]
    MIDDLE_FLBR = MOTOR_CONSTANTS["MIDDLE_FLBR"]["port"]
    MIDDLE_FMBM = MOTOR_CONSTANTS["MIDDLE_FMBM"]["port"]

    WRIST = MOTOR_CONSTANTS["WRIST"]["port"]

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

    POSITION = auto()
    ORIENTATION = auto()

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

    POSE_X = auto()
    POSE_Y = auto()
    POSE_Z = auto()
    POSE_ROLL = auto()
    POSE_PITCH = auto()
    POSE_YAW = auto()

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

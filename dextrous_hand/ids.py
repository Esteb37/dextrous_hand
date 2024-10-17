#!/usr/bin/env python3

from enum import Enum, auto
from numpy import pi

class MOTORS(Enum):
    """
    Motor ports
    """
    PINKY_TRBL = 0
    PINKY_TLBR = 1
    PINKY_TMBM = 2

    RING_TRBL = 3
    RING_TLBR = 4
    RING_TMBM = 5

    MIDDLE_TRBL = 6
    MIDDLE_TLBR = 7
    MIDDLE_TMBM = 8

    INDEX_TRBL = 9
    INDEX_TLBR = 10
    INDEX_TMBM = 11

    THUMB_TRBL = 12
    THUMB_TLBR = 13
    THUMB_TMBM = 14

    WRIST = 15

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

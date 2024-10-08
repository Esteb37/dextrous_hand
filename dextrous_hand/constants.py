#!/usr/bin/env python3

from enum import Enum, auto

class IDS(Enum):
    """
    Subsystem IDs.
    Each subsystem has a unique ID, determined automatically by the Enum class.
    A subsystem is a unique element of the robot that can be controlled independently.
    """

    THUMB = auto()
    INDEX = auto()
    MIDDLE = auto()
    RING = auto()
    PINKY = auto()
    ABDUCTION = auto()
    WRIST = auto()

MOTOR_PORTS = {
    IDS.THUMB: [0, 1, 2],
    IDS.INDEX: [3, 4, 5],
    IDS.MIDDLE: [6, 7, 8],
    IDS.RING: [9, 10, 11],
    IDS.PINKY: [12, 13, 14],
    IDS.ABDUCTION: [15],
    IDS.WRIST: [16]
}
"""
Motor ports for each subsystem.
Subsystems with multiple motors have a list of motor ports.
"""

MOTOR_LIMITS = {
    IDS.THUMB: [[-3.14, 3.14],
                [-3.14, 3.14],
                [-3.14, 3.14]],
    IDS.INDEX: [[-3.14, 3.14],
                [-3.14, 3.14],
                [-3.14, 3.14]],
    IDS.MIDDLE:[[-3.14, 3.14],
                [-3.14, 3.14],
                [-3.14, 3.14]],
    IDS.RING:  [[-3.14, 3.14],
                [-3.14, 3.14],
                [-3.14, 3.14]],
    IDS.PINKY: [[-3.14, 3.14],
                [-3.14, 3.14],
                [-3.14, 3.14]],
    IDS.ABDUCTION: [[-3.14, 3.14]],
    IDS.WRIST: [[-3.14, 3.14]]
}
"""
Hard limits for the motors to prevent overextension.
"""
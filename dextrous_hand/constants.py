#!/usr/bin/env python3

from enum import Enum, auto
from numpy import pi as PI

IS_SIMULATION = True # Set to True if the motor controller is not connected
DEVICENAMES = ["/dev/ttyUSB0", "/dev/ttyUSB1"]

class MOTORS(Enum):
    THUMB_0 = 0
    THUMB_1 = 1
    THUMB_2 = 2
    INDEX_0 = 3
    INDEX_1 = 4
    INDEX_2 = 5
    MIDDLE_0 = 6
    MIDDLE_1 = 7
    MIDDLE_2 = 8
    RING_0 = 9
    RING_1 = 10
    RING_2 = 11
    PINKY_0 = 12
    PINKY_1 = 13
    PINKY_2 = 14
    ABDUCTION = 15
    WRIST = 16
"""
Motor ports
"""

FULL_RANGE = [-PI, PI]
MOTOR_LIMITS = {
    MOTORS.THUMB_0: FULL_RANGE,
    MOTORS.THUMB_1: FULL_RANGE,
    MOTORS.THUMB_2: FULL_RANGE,
    MOTORS.INDEX_0: FULL_RANGE,
    MOTORS.INDEX_1: FULL_RANGE,
    MOTORS.INDEX_2: FULL_RANGE,
    MOTORS.MIDDLE_0: FULL_RANGE,
    MOTORS.MIDDLE_1: FULL_RANGE,
    MOTORS.MIDDLE_2: FULL_RANGE,
    MOTORS.RING_0: FULL_RANGE,
    MOTORS.RING_1: FULL_RANGE,
    MOTORS.RING_2: FULL_RANGE,
    MOTORS.PINKY_0: FULL_RANGE,
    MOTORS.PINKY_1: FULL_RANGE,
    MOTORS.PINKY_2: FULL_RANGE,
    MOTORS.ABDUCTION: FULL_RANGE,
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
    THUMB = auto()
    INDEX = auto()
    MIDDLE = auto()
    RING = auto()
    PINKY = auto()
    ABDUCTION = auto()
    WRIST = auto()

class JOINTS(Enum):
    """
    Joint IDs.
    Each joint has a unique ID, determined automatically by the Enum class.
    A joint is a component of a subsystem that can be controlled independently.
    """
    THUMB_BASE = auto()
    THUMB_MID = auto()
    THUMB_TIP = auto()
    INDEX_BASE = auto()
    INDEX_MID = auto()
    INDEX_TIP = auto()
    MIDDLE_BASE = auto()
    MIDDLE_MID = auto()
    MIDDLE_TIP = auto()
    RING_BASE = auto()
    RING_MID = auto()
    RING_TIP = auto()
    PINKY_BASE = auto()
    PINKY_MID = auto()
    PINKY_TIP = auto()
    ABDUCTION = auto()
    WRIST = auto()

JOINT_MOTORS = {
    JOINTS.THUMB_BASE: [MOTORS.THUMB_0],
    JOINTS.THUMB_MID: [MOTORS.THUMB_1],
    JOINTS.THUMB_TIP: [MOTORS.THUMB_2],
    JOINTS.INDEX_BASE: [MOTORS.INDEX_0],
    JOINTS.INDEX_MID: [MOTORS.INDEX_1],
    JOINTS.INDEX_TIP: [MOTORS.INDEX_2],
    JOINTS.MIDDLE_BASE: [MOTORS.MIDDLE_0],
    JOINTS.MIDDLE_MID: [MOTORS.MIDDLE_1],
    JOINTS.MIDDLE_TIP: [MOTORS.MIDDLE_2],
    JOINTS.RING_BASE: [MOTORS.RING_0],
    JOINTS.RING_MID: [MOTORS.RING_1],
    JOINTS.RING_TIP: [MOTORS.RING_2],
    JOINTS.PINKY_BASE: [MOTORS.PINKY_0],
    JOINTS.PINKY_MID: [MOTORS.PINKY_1],
    JOINTS.PINKY_TIP: [MOTORS.PINKY_2],
    JOINTS.ABDUCTION: [MOTORS.ABDUCTION],
    JOINTS.WRIST: [MOTORS.WRIST]
}
"""
Motor ports for each joint
"""

SUBSYSTEM_JOINTS = {
    SUBSYSTEMS.THUMB: [JOINTS.THUMB_BASE, JOINTS.THUMB_MID, JOINTS.THUMB_TIP],
    SUBSYSTEMS.INDEX: [JOINTS.INDEX_BASE, JOINTS.INDEX_MID, JOINTS.INDEX_TIP],
    SUBSYSTEMS.MIDDLE: [JOINTS.MIDDLE_BASE, JOINTS.MIDDLE_MID, JOINTS.MIDDLE_TIP],
    SUBSYSTEMS.RING: [JOINTS.RING_BASE, JOINTS.RING_MID, JOINTS.RING_TIP],
    SUBSYSTEMS.PINKY: [JOINTS.PINKY_BASE, JOINTS.PINKY_MID, JOINTS.PINKY_TIP],
    SUBSYSTEMS.ABDUCTION: [JOINTS.ABDUCTION],
    SUBSYSTEMS.WRIST: [JOINTS.WRIST]
}
"""
Joints for each subsystem
"""

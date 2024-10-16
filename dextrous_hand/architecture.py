#!/usr/bin/env python3

import dextrous_hand.ids as ids
from dextrous_hand.FingerJoints import PIP, DIP, MCP, ABD
from dextrous_hand.Wrist import WristJoint
from dextrous_hand.Motor import Motor

SUBSYSTEM_JOINTS = {
    ids.SUBSYSTEMS.PINKY: [ABD(ids.JOINTS.PINKY_ABD),
                           MCP(ids.JOINTS.PINKY_MCP),
                           PIP(ids.JOINTS.PINKY_PIP),
                           DIP(ids.JOINTS.PINKY_DIP)],

    ids.SUBSYSTEMS.RING: [ABD(ids.JOINTS.RING_ABD),
                          MCP(ids.JOINTS.RING_MCP),
                          PIP(ids.JOINTS.RING_PIP),
                          DIP(ids.JOINTS.RING_DIP)],

    ids.SUBSYSTEMS.MIDDLE: [ABD(ids.JOINTS.MIDDLE_ABD),
                            MCP(ids.JOINTS.MIDDLE_MCP),
                            PIP(ids.JOINTS.MIDDLE_PIP),
                            DIP(ids.JOINTS.MIDDLE_DIP)],

    ids.SUBSYSTEMS.INDEX: [ABD(ids.JOINTS.INDEX_ABD),
                           MCP(ids.JOINTS.INDEX_MCP),
                           PIP(ids.JOINTS.INDEX_PIP),
                           DIP(ids.JOINTS.INDEX_DIP)],

    ids.SUBSYSTEMS.THUMB: [ABD(ids.JOINTS.THUMB_ABD),
                           MCP(ids.JOINTS.THUMB_MCP),
                           PIP(ids.JOINTS.THUMB_PIP),
                           DIP(ids.JOINTS.THUMB_DIP)],

    ids.SUBSYSTEMS.WRIST: [WristJoint(ids.JOINTS.WRIST)]
}
"""
Joints for each subsystem
"""

SUBSYSTEM_MOTORS = {
    ids.SUBSYSTEMS.PINKY :  [Motor(ids.MOTORS.PINKY_UMBM),
                             Motor(ids.MOTORS.PINKY_URBL),
                             Motor(ids.MOTORS.PINKY_ULBR)],

    ids.SUBSYSTEMS.RING :   [Motor(ids.MOTORS.RING_UMBM),
                             Motor(ids.MOTORS.RING_URBL),
                             Motor(ids.MOTORS.RING_ULBR)],

    ids.SUBSYSTEMS.MIDDLE : [Motor(ids.MOTORS.MIDDLE_UMBM),
                             Motor(ids.MOTORS.MIDDLE_URBL),
                             Motor(ids.MOTORS.MIDDLE_ULBR)],

    ids.SUBSYSTEMS.INDEX :  [Motor(ids.MOTORS.INDEX_UMBM),
                             Motor(ids.MOTORS.INDEX_URBL),
                             Motor(ids.MOTORS.INDEX_ULBR)],

    ids.SUBSYSTEMS.THUMB :  [Motor(ids.MOTORS.THUMB_UMBM),
                             Motor(ids.MOTORS.THUMB_URBL),
                             Motor(ids.MOTORS.THUMB_ULBR)],

    ids.SUBSYSTEMS.WRIST :  [Motor(ids.MOTORS.WRIST)]
}
"""
Motors for each subsystem
"""

JOINT_MOTORS = {
    ids.JOINTS.PINKY_ABD: [Motor(ids.MOTORS.PINKY_URBL), Motor(ids.MOTORS.PINKY_ULBR)],
    ids.JOINTS.PINKY_MCP: [Motor(ids.MOTORS.PINKY_URBL), Motor(ids.MOTORS.PINKY_ULBR)],
    ids.JOINTS.PINKY_PIP: [Motor(ids.MOTORS.PINKY_UMBM)],

    ids.JOINTS.RING_ABD: [Motor(ids.MOTORS.RING_URBL), Motor(ids.MOTORS.RING_ULBR)],
    ids.JOINTS.RING_MCP: [Motor(ids.MOTORS.RING_URBL), Motor(ids.MOTORS.RING_ULBR)],
    ids.JOINTS.RING_PIP: [Motor(ids.MOTORS.RING_UMBM)],

    ids.JOINTS.MIDDLE_ABD: [Motor(ids.MOTORS.MIDDLE_URBL), Motor(ids.MOTORS.MIDDLE_ULBR)],
    ids.JOINTS.MIDDLE_MCP: [Motor(ids.MOTORS.MIDDLE_URBL), Motor(ids.MOTORS.MIDDLE_ULBR)],
    ids.JOINTS.MIDDLE_PIP: [Motor(ids.MOTORS.MIDDLE_UMBM)],

    ids.JOINTS.INDEX_ABD: [Motor(ids.MOTORS.INDEX_URBL), Motor(ids.MOTORS.INDEX_ULBR)],
    ids.JOINTS.INDEX_MCP: [Motor(ids.MOTORS.INDEX_URBL), Motor(ids.MOTORS.INDEX_ULBR)],
    ids.JOINTS.INDEX_PIP: [Motor(ids.MOTORS.INDEX_UMBM)],

    ids.JOINTS.THUMB_ABD: [Motor(ids.MOTORS.THUMB_URBL), Motor(ids.MOTORS.THUMB_ULBR)],
    ids.JOINTS.THUMB_MCP: [Motor(ids.MOTORS.THUMB_URBL), Motor(ids.MOTORS.THUMB_ULBR)],
    ids.JOINTS.THUMB_PIP: [Motor(ids.MOTORS.THUMB_UMBM)],

    ids.JOINTS.WRIST: [Motor(ids.MOTORS.WRIST)]}
"""
Motor composition for each joint
"""
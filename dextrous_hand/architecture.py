#!/usr/bin/env python3

from dextrous_hand.ids import *
from dextrous_hand.FingerJoints import PIP, DIP, MCP, ABD
from dextrous_hand.WristJoint import WristJoint


"""
    TRLB = Top right, bottom left
    TLBR = Top left, bottom right
    TMBM = Top middle, bottom middle
"""

SUBSYSTEM_MOTORS = {
    SUBSYSTEMS.PINKY :  [MOTORS.PINKY_TRBL,
                         MOTORS.PINKY_TLBR,
                         MOTORS.PINKY_TMBM
                         ],

    SUBSYSTEMS.RING :   [MOTORS.RING_TRBL,
                         MOTORS.RING_TLBR,
                         MOTORS.RING_TMBM
                         ],

    SUBSYSTEMS.MIDDLE : [MOTORS.MIDDLE_TRBL,
                         MOTORS.MIDDLE_TLBR,
                         MOTORS.MIDDLE_TMBM
                         ],

    SUBSYSTEMS.INDEX :  [MOTORS.INDEX_TRBL,
                         MOTORS.INDEX_TLBR,
                         MOTORS.INDEX_TMBM
                         ],

    SUBSYSTEMS.THUMB :  [MOTORS.THUMB_TRBL,
                         MOTORS.THUMB_TLBR,
                         MOTORS.THUMB_TMBM
                         ],

    SUBSYSTEMS.WRIST :  [MOTORS.WRIST]
}
"""
Motors for each subsystem
"""

JOINT_MOTORS = {
    JOINTS.PINKY_ABD: [MOTORS.PINKY_TRBL, MOTORS.PINKY_TLBR],
    JOINTS.PINKY_MCP: [MOTORS.PINKY_TRBL, MOTORS.PINKY_TLBR],
    JOINTS.PINKY_PIP: [MOTORS.PINKY_TMBM],
    JOINTS.PINKY_DIP: [MOTORS.PINKY_TMBM],

    JOINTS.RING_ABD: [MOTORS.RING_TRBL, MOTORS.RING_TLBR],
    JOINTS.RING_MCP: [MOTORS.RING_TRBL, MOTORS.RING_TLBR],
    JOINTS.RING_PIP: [MOTORS.RING_TMBM],
    JOINTS.RING_DIP: [MOTORS.RING_TMBM],

    JOINTS.MIDDLE_ABD: [MOTORS.MIDDLE_TRBL, MOTORS.MIDDLE_TLBR],
    JOINTS.MIDDLE_MCP: [MOTORS.MIDDLE_TRBL, MOTORS.MIDDLE_TLBR],
    JOINTS.MIDDLE_PIP: [MOTORS.MIDDLE_TMBM],
    JOINTS.MIDDLE_DIP: [MOTORS.MIDDLE_TMBM],

    JOINTS.INDEX_ABD: [MOTORS.INDEX_TRBL, MOTORS.INDEX_TLBR],
    JOINTS.INDEX_MCP: [MOTORS.INDEX_TRBL, MOTORS.INDEX_TLBR],
    JOINTS.INDEX_PIP: [MOTORS.INDEX_TMBM],
    JOINTS.INDEX_DIP: [MOTORS.INDEX_TMBM],

    JOINTS.THUMB_ABD: [MOTORS.THUMB_TRBL, MOTORS.THUMB_TLBR],
    JOINTS.THUMB_MCP: [MOTORS.THUMB_TRBL, MOTORS.THUMB_TLBR],
    JOINTS.THUMB_PIP: [MOTORS.THUMB_TMBM],
    JOINTS.THUMB_DIP: [MOTORS.THUMB_TMBM],

    JOINTS.WRIST: [MOTORS.WRIST]}
"""
Motor composition for each joint
"""

SUBSYSTEM_JOINTS = {
    SUBSYSTEMS.PINKY: [ABD(JOINTS.PINKY_ABD),
                       MCP(JOINTS.PINKY_MCP),
                       PIP(JOINTS.PINKY_PIP),
                       DIP(JOINTS.PINKY_DIP)],

    SUBSYSTEMS.RING: [ABD(JOINTS.RING_ABD),
                      MCP(JOINTS.RING_MCP),
                      PIP(JOINTS.RING_PIP),
                      DIP(JOINTS.RING_DIP)],

    SUBSYSTEMS.MIDDLE: [ABD(JOINTS.MIDDLE_ABD),
                        MCP(JOINTS.MIDDLE_MCP),
                        PIP(JOINTS.MIDDLE_PIP),
                        DIP(JOINTS.MIDDLE_DIP)],

    SUBSYSTEMS.INDEX: [ABD(JOINTS.INDEX_ABD),
                       MCP(JOINTS.INDEX_MCP),
                       PIP(JOINTS.INDEX_PIP),
                       DIP(JOINTS.INDEX_DIP)],

    SUBSYSTEMS.THUMB: [ABD(JOINTS.THUMB_ABD),
                       MCP(JOINTS.THUMB_MCP),
                       PIP(JOINTS.THUMB_PIP),
                       DIP(JOINTS.THUMB_DIP)],

    SUBSYSTEMS.WRIST: [WristJoint(JOINTS.WRIST)]
}
"""
Joints for each subsystem
"""
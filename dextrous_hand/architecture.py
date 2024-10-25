#!/usr/bin/env python3

from dextrous_hand.ids import *
from dextrous_hand.FingerJoints import PIP, DIP, MCP, ABD
from dextrous_hand.WristJoint import WristJoint


"""
    TRLB = Front right, back left
    FLBR = Front left, back right
    FMBM = Front middle, back middle
"""

SUBSYSTEM_MOTORS = {
    SUBSYSTEMS.PINKY :  [MOTORS.PINKY_FRBL,
                         MOTORS.PINKY_FLBR,
                         MOTORS.PINKY_FMBM
                         ],

    SUBSYSTEMS.RING :   [MOTORS.RING_FRBL,
                         MOTORS.RING_FLBR,
                         MOTORS.RING_FMBM
                         ],

    SUBSYSTEMS.MIDDLE : [MOTORS.MIDDLE_FRBL,
                         MOTORS.MIDDLE_FLBR,
                         MOTORS.MIDDLE_FMBM
                         ],

    SUBSYSTEMS.INDEX :  [MOTORS.INDEX_FRBL,
                         MOTORS.INDEX_FLBR,
                         MOTORS.INDEX_FMBM
                         ],

    SUBSYSTEMS.THUMB :  [MOTORS.THUMB_FRBL,
                         MOTORS.THUMB_FLBR,
                         MOTORS.THUMB_FMBM
                         ],

    SUBSYSTEMS.WRIST :  [MOTORS.WRIST]
}
"""
Motors for each subsystem
"""

JOINT_MOTORS = {
    JOINTS.PINKY_ABD: [MOTORS.PINKY_FRBL, MOTORS.PINKY_FLBR],
    JOINTS.PINKY_MCP: [MOTORS.PINKY_FRBL, MOTORS.PINKY_FLBR],
    JOINTS.PINKY_PIP: [MOTORS.PINKY_FMBM],
    JOINTS.PINKY_DIP: [MOTORS.PINKY_FMBM],

    JOINTS.RING_ABD: [MOTORS.RING_FRBL, MOTORS.RING_FLBR],
    JOINTS.RING_MCP: [MOTORS.RING_FRBL, MOTORS.RING_FLBR],
    JOINTS.RING_PIP: [MOTORS.RING_FMBM],
    JOINTS.RING_DIP: [MOTORS.RING_FMBM],

    JOINTS.MIDDLE_ABD: [MOTORS.MIDDLE_FRBL, MOTORS.MIDDLE_FLBR],
    JOINTS.MIDDLE_MCP: [MOTORS.MIDDLE_FRBL, MOTORS.MIDDLE_FLBR],
    JOINTS.MIDDLE_PIP: [MOTORS.MIDDLE_FMBM],
    JOINTS.MIDDLE_DIP: [MOTORS.MIDDLE_FMBM],

    JOINTS.INDEX_ABD: [MOTORS.INDEX_FRBL, MOTORS.INDEX_FLBR],
    JOINTS.INDEX_MCP: [MOTORS.INDEX_FRBL, MOTORS.INDEX_FLBR],
    JOINTS.INDEX_PIP: [MOTORS.INDEX_FMBM],
    JOINTS.INDEX_DIP: [MOTORS.INDEX_FMBM],

    JOINTS.THUMB_ABD: [MOTORS.THUMB_FRBL, MOTORS.THUMB_FLBR],
    JOINTS.THUMB_MCP: [MOTORS.THUMB_FRBL, MOTORS.THUMB_FLBR],
    JOINTS.THUMB_PIP: [MOTORS.THUMB_FMBM],
    JOINTS.THUMB_DIP: [MOTORS.THUMB_FMBM],

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
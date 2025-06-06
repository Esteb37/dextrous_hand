#!/usr/bin/env python3

from dextrous_hand.utils.ids import *
from dextrous_hand.joints.FingerJoints import *
from dextrous_hand.joints.Joint import ArmJoint
from dextrous_hand.joints.WristJoint import WristJoint

def find_parent_subsystem(id : JOINTS | MOTORS) -> SUBSYSTEMS | None:
    """
    Find the parent subsystem of a joint
    """
    if type(id) == JOINTS:
        for subsystem in SUBSYSTEM_JOINTS:
            ids = [joint.id for joint in SUBSYSTEM_JOINTS[subsystem]]
            if id in ids:
                return subsystem

    if type(id) == MOTORS:
        for subsystem in SUBSYSTEM_MOTORS:
            if id in SUBSYSTEM_MOTORS[subsystem]:
                return subsystem


def find_parent_joint(id : MOTORS) -> JOINTS | None:
    """
    Find the parent joint of a motor
    """
    for joint in JOINT_MOTORS:
        if id in JOINT_MOTORS[joint]:
            return joint

def find_joint_index(parent : SUBSYSTEMS, joint_id : JOINTS) -> int:
    """
    Find the index of a joint in a list of joints
    """
    joints = SUBSYSTEM_JOINTS[parent]
    joint_ids = [joint.id for joint in joints]
    return joint_ids.index(joint_id)

def find_motor_index(parent : SUBSYSTEMS | JOINTS, motor_id : MOTORS) -> int:
    """
    Find the index of a motor in a list of motors
    """

    if type(parent) == SUBSYSTEMS:
        motors = SUBSYSTEM_MOTORS[parent]
    elif type(parent) == JOINTS:
        motors = JOINT_MOTORS[parent]

    return motors.index(motor_id)

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

    SUBSYSTEMS.THUMB :  [MOTORS.THUMB_MCP,
                         MOTORS.THUMB_ABD,
                         MOTORS.THUMB_PIP,
                         MOTORS.THUMB_DIP
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

    JOINTS.THUMB_ABD: [MOTORS.THUMB_ABD],
    JOINTS.THUMB_MCP: [MOTORS.THUMB_MCP],
    JOINTS.THUMB_PIP: [MOTORS.THUMB_PIP],
    JOINTS.THUMB_DIP: [MOTORS.THUMB_DIP],

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

    SUBSYSTEMS.THUMB: [THUMB_MCP(JOINTS.THUMB_MCP),
                       THUMB_ABD(JOINTS.THUMB_ABD),
                       THUMB_PIP(JOINTS.THUMB_PIP),
                       THUMB_DIP(JOINTS.THUMB_DIP)],

    SUBSYSTEMS.WRIST: [WristJoint(JOINTS.WRIST)],

    SUBSYSTEMS.POSITION: [ArmJoint(JOINTS.POS_X),
                          ArmJoint(JOINTS.POS_Y),
                          ArmJoint(JOINTS.POS_Z)],

    SUBSYSTEMS.ORIENTATION: [ArmJoint(JOINTS.ORIENT_X),
                             ArmJoint(JOINTS.ORIENT_Y),
                             ArmJoint(JOINTS.ORIENT_Z),
                             ArmJoint(JOINTS.ORIENT_W)
                             ],


}
"""
Joints for each subsystem
"""
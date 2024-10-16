#!/usr/bin/env python3

from dextrous_hand.Subsystem import Subsystem
import dextrous_hand.ids as ids
from dextrous_hand.Joint import Joint

class WristJoint(Joint):
    def __init__(self, joint_id : ids.JOINTS):
        """
        params
            id [JOINTS]: the joint's id
        """
        super().__init__(joint_id)

    def motors2joint(self, motor_angles):
        """
        TODO: Implement this method.
              It's probably just the angle of the motor through a gear ratio.
        """
        return motor_angles[0]

class Wrist(Subsystem):
    def __init__(self, id : ids.SUBSYSTEMS):
        """
        params
            id [SUBSYSTEMS]: the wrist's id
        """
        super().__init__(id)

    def joints2motors(self, joint_angles):
        """
        Map the angle of the joint to the angle of the motor
        TODO: Implement this method
              It's probably just the angle of the joint through a gear ratio.
        """
        joint_angle = joint_angles[0]
        return joint_angle

# Singleton instance
WRIST = Wrist(ids.SUBSYSTEMS.WRIST)

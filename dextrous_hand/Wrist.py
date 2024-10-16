#!/usr/bin/env python3

from dextrous_hand.Subsystem import Subsystem
import dextrous_hand.ids as ids
from dextrous_hand.WristJoint import WristJoint

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

#!/usr/bin/env python3

from dextrous_hand.Joint import Joint
import dextrous_hand.ids as ids

class WristJoint(Joint):

    GEAR_RATIO = 1.0

    def __init__(self, id : ids.JOINTS):
        """
        params
            id [JOINTS]: the joint's id
        """
        super().__init__(id)

    def motors2joint(self, motor_angles):
        """
        Map the angle of the motor to the angle of the joint
        TODO: Implement this method, probably just the angle of the motor through a gear ratio.
        """
        return motor_angles[0] * self.GEAR_RATIO
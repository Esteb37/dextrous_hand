#!/usr/bin/env python3

import dextrous_hand.utils.ids as ids
from dextrous_hand.joints.Joint import Joint

class WristJoint(Joint):

    GEAR_RATIO = 1/4.0

    def __init__(self, id : ids.JOINTS):
        """
        params
            id [JOINTS]: the joint's id
        """
        super().__init__(id)

    def motors2joint(self, motor_angles):
        """
        Map the angle of the motor to the angle of the joint
        """
        return motor_angles[0] * self.GEAR_RATIO
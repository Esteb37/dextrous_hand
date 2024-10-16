#!/usr/bin/env python3

from dextrous_hand.Subsystem import Subsystem
import dextrous_hand.ids as ids

class Finger(Subsystem):
    def __init__(self, id : ids.SUBSYSTEMS):
        """
        params
            id [SUBSYSTEMS]: the finger's id
        """
        super().__init__(id)
        self.ABD = self.joints[0]
        self.MCP = self.joints[1]
        self.PIP = self.joints[2]

    def joints2motors(self, joint_angles):
        """
        Map the angles of the joints to the angles of the motors
        TODO: Implement this method
        """

        assert len(joint_angles) == 3

        return joint_angles

# Singleton instances
PINKY = Finger(ids.SUBSYSTEMS.PINKY)
RING = Finger(ids.SUBSYSTEMS.RING)
MIDDLE = Finger(ids.SUBSYSTEMS.MIDDLE)
INDEX = Finger(ids.SUBSYSTEMS.INDEX)
THUMB = Finger(ids.SUBSYSTEMS.THUMB)

FINGERS = [PINKY, RING, MIDDLE, INDEX, THUMB]
"""
 A collection of fingers for easy iteration.
"""
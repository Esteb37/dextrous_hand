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

    def joints2motors(self, joint_angles) -> list[float]:
        """
        Map the angles of the joints to the angles of the motors.
        Note that the number of angles that can be set is 3 and not 4 because
        DIP is not settable.

        TODO: Implement this method
        """

        assert len(joint_angles) == 3

        return joint_angles

    @property
    def ABD(self):
        return self.find_joint("ABD")

    @property
    def MCP(self):
        return self.find_joint("MCP")

    @property
    def PIP(self):
        return self.find_joint("PIP")

    @property
    def DIP(self):
        return self.find_joint("DIP")

    @property
    def URBL(self):
        return self.find_motor("URBL")

    @property
    def ULBR(self):
        return self.find_motor("ULBR")

    @property
    def UMBM(self):
        return self.find_motor("UMBM")

class Thumb(Finger):
    """
    For now, the thumb is the same as a finger.
    TODO: Implement thumb-specific methods
    """
    def __init__(self, id : ids.SUBSYSTEMS):
        """
        params
            id [SUBSYSTEMS]: the thumb's id
        """
        super().__init__(id)

    def joints2motors(self, joint_angles) -> list[float]:
        """
        Map the angles of the joints to the angles of the motors.
        Note that the number of angles that can be set is 3 and not 4 because
        DIP is not settable.

        TODO: Implement this method
        """
        assert len(joint_angles) == 3

        return joint_angles

# Singleton instances
PINKY = Finger(ids.SUBSYSTEMS.PINKY)
RING = Finger(ids.SUBSYSTEMS.RING)
MIDDLE = Finger(ids.SUBSYSTEMS.MIDDLE)
INDEX = Finger(ids.SUBSYSTEMS.INDEX)
THUMB = Thumb(ids.SUBSYSTEMS.THUMB)

FINGERS = [PINKY, RING, MIDDLE, INDEX, THUMB]
"""
 A collection of fingers for easy iteration.
"""
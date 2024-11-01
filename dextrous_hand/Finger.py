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

        return [joint_angles[1] - joint_angles[0] / 2, joint_angles[1] + joint_angles[0] / 2, joint_angles[2]]

    @property
    def ABD(self):
        return self.get_joint("ABD")

    @property
    def MCP(self):
        return self.get_joint("MCP")

    @property
    def PIP(self):
        return self.get_joint("PIP")

    @property
    def DIP(self):
        return self.get_joint("DIP")

    @property
    def FRBL(self):
        return self.get_motor("FRBL")

    @property
    def FLBR(self):
        return self.get_motor("FLBR")

    @property
    def FMBM(self):
        return self.get_motor("FMBM")

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

FINGERS = [Finger(id) for id in ids.SUBSYSTEMS if id != ids.SUBSYSTEMS.WRIST]
FINGERS[ids.SUBSYSTEMS.THUMB.value] = Thumb(ids.SUBSYSTEMS.THUMB)
"""
 A collection of fingers for easy iteration.
"""

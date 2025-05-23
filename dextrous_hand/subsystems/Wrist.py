#!/usr/bin/env python3

from dextrous_hand.subsystems.Subsystem import Subsystem
import dextrous_hand.utils.ids as ids

class Wrist(Subsystem):
    def __init__(self, id : ids.SUBSYSTEMS):
        """
        params
            id [SUBSYSTEMS]: the wrist's id
        """
        super().__init__(id)

    def joints2motors(self, joint_angles) -> list[float]:
        """
        Map the angle of the joint to the angle of the motor
        """
        return [joint_angles[0] / self.JOINT.GEAR_RATIO]

    @property
    def JOINT(self):
        return self.joints[0]

    @property
    def MOTOR(self):
        return self.motors[0]

# Singleton instance
WRIST = Wrist(ids.SUBSYSTEMS.WRIST)

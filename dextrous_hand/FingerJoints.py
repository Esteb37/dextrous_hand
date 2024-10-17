#!/usr/bin/env python3

from dextrous_hand.Joint import Joint
import dextrous_hand.ids as ids


class ABD(Joint):
    """
    The abduction joint of a finger.
    """
    def __init__(self, joint_id : ids.JOINTS):
        """
        params
            id [JOINTS]: the joint's id
        """
        super().__init__(joint_id)

    def motors2joint(self, motor_angles):
        """
        Map the angles of the motor(s) to the angle of the joint
        TODO: Implement this method
        """
        assert len(motor_angles) == len(self.motors)

        return self.get_motor("TRBL").read()

class MCP(Joint):
    """
    The metacarpophalangeal joint of a finger.
    """
    def __init__(self, joint_id : ids.JOINTS):
        """
        params
            id [JOINTS]: the joint's id
        """
        super().__init__(joint_id)

    def motors2joint(self, motor_angles):
        """
        Map the angles of the motor(s) to the angle of the joint
        TODO: Implement this method
        """
        assert len(motor_angles) == len(self.motors)

        return self.get_motor("TLBR").read()


class PIP(Joint):
    """
    The proximal interphalangeal joint of a finger.
    """
    def __init__(self, joint_id : ids.JOINTS):
        """
        params
            id [JOINTS]: the joint's id
        """
        super().__init__(joint_id)

    def motors2joint(self, motor_angles):
        """
        Map the angles of the motor(s) to the angle of the joint
        TODO: Implement this method
        """
        assert len(motor_angles) == len(self.motors)

        return self.get_motor("TMBM").read()

class DIP(Joint):
    """
    The distal interphalangeal joint of a finger.
    """
    def __init__(self, joint_id : ids.JOINTS):
        """
        params
            id [JOINTS]: the joint's id
        """
        super().__init__(joint_id)

    def motors2joint(self, motor_angles):
        """
        TODO: Implement this method
              Should be the same as the PIP joint, maybe with an offset or a scaling factor
        """
        assert len(motor_angles) == len(self.motors)

        return self.get_motor("TMBM").read()

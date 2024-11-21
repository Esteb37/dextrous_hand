#!/usr/bin/env python3

import dextrous_hand.utils.ids as ids
from dextrous_hand.joints.Joint import Joint

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

        return self.get_motor("FLBR").read() -  self.get_motor("FRBL").read()

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

        return (self.get_motor("FLBR").read() + self.get_motor("FRBL").read()) / 2


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

        return self.get_motor("FMBM").read()

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

        return self.get_motor("FMBM").read()


class THUMB_ABD(Joint):
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

        return self.get_motor("ABD").read()

class THUMB_MCP(Joint):
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

        return self.get_motor("MCP").read()

class THUMB_PIP(Joint):
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

        return self.get_motor("PIP").read()

class THUMB_DIP(Joint):
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

        return self.get_motor("DIP").read()
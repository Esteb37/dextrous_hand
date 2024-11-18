#!/usr/bin/env python3

from abc import ABC, abstractmethod

import dextrous_hand.utils.ids as ids
from dextrous_hand.motors.Motor import Motor

class Subsystem(ABC):
    """
    A subsystem is any collection of joints that can be controlled together.
    It is an abstract class so that each finger / wrist defines how to write to its joints.
    """

    # For the singleton pattern
    _instances = {}

    def __new__(cls, subsystem_id, *args, **kwargs):
        """
        Singleton pattern. Make sure only one instance of each Subsystem is created.
        If it has already been created, return the existing instance.
        """
        if subsystem_id not in cls._instances:
            cls._instances[subsystem_id] = super(Subsystem, cls).__new__(cls)
        return cls._instances[subsystem_id]

    def __init__(self, subsystem_id : ids.SUBSYSTEMS):
        """
        params
            subsystem_id [SUBSYSTEMS]: the subsystem's id
        """
        # Avoid reinitialization if the instance already exists
        if hasattr(self, 'initialized') and self.initialized:
            return

        self.id = subsystem_id


        from dextrous_hand.utils.architecture import SUBSYSTEM_JOINTS, SUBSYSTEM_MOTORS

        # Check if the subsystem has joints in the architecture.py file
        if self.id not in SUBSYSTEM_JOINTS or len(SUBSYSTEM_JOINTS[self.id]) == 0:
            raise Exception("Subsystem " + self.id.name + " has no joints")

        self.joints = SUBSYSTEM_JOINTS[self.id]
        self.joint_count = len(self.joints)

        self.motors = [Motor(motor_id) for motor_id in SUBSYSTEM_MOTORS[self.id]]

        # For the singleton pattern
        self.initialized = True

    @abstractmethod
    def joints2motors(self, joint_angles) -> list[float]:
        """
        Map the angles of the joints to the angles of the motors
        Each subclass should uniquely implement this method

        params
            joint_angles: a list of joint angles

        returns
            a list of motor angles
        """
        pass

    def restrict_joint_angles(self, joint_angles):
        return joint_angles

    def write(self, joint_angles):
        """
        Map joint angles to motor angles and write to the motors

        params
            joint_angles: a list of joint angles
        """

        # Allow for single-joint subsystems to be controlled with a single float
        if type(joint_angles) == float:
            joint_angles = [joint_angles]

        joint_angles = self.restrict_joint_angles(joint_angles)

        for i, angle in enumerate(joint_angles):
            self.joints[i].target = angle

        motor_angles = self.joints2motors(joint_angles)

        for motor, angle in zip(self.motors, motor_angles): # type: ignore
            motor.write(angle)

    def read(self):
        """
        returns
            The positions of all joints in the subsystem
        """
        return [joint.read() for joint in self.joints]

    def at_target(self):
        """
        returns
            True if all motors in the subsystem are at their target angles
        """
        return all([motor.at_angle() for motor in self.motors])

    def get_joint(self, partial_id):
        """
        Tries to find a joint whose name contains the provided substring

        params
            partial_id [JOINTS]: a substring of the joint's id

        returns
            The joint if found

        raises
            Exception: if no joint is found with the provided substring
        """

        joints = [joint for joint in self.joints if partial_id in joint.id.name]
        if len(joints) == 0:
            raise Exception(self.id.name + " has no joint with id containing " + partial_id)
        return joints[0]

    def get_motor(self, partial_id):
        """
        Tries to find a motor whose name contains the provided substring

        params
            partial_id [MOTORS]: a substring of the motor's id

        returns
            The motor if found

        raises
            Exception: if no motor is found with the provided substring
        """

        motors = [motor for motor in self.motors if partial_id in motor.id.name]
        if len(motors) == 0:
            raise Exception(self.id.name + " has no motor with id containing " + partial_id)
        return motors[0]

    def __str__(self):
        formatted_values = [f"{value:.3f}" for value in self.read()]
        return self.id.name + ": " + str(formatted_values)

    def __getitem__(self, joint_index):
        """
        To support 'obj[joint_index]' for getting joints
        """
        return self.joints[joint_index]

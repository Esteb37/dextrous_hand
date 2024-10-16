#!/usr/bin/env python3

import dextrous_hand.ids as ids
from dextrous_hand.Motor import Motor
from abc import ABC, abstractmethod

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


        from dextrous_hand.architecture import SUBSYSTEM_JOINTS, SUBSYSTEM_MOTORS

        # Check if the subsystem has joints in the architecture.py file
        if self.id not in SUBSYSTEM_JOINTS or len(SUBSYSTEM_JOINTS[self.id]) == 0:
            raise Exception("Subsystem " + self.id.name + " has no joints")

        self.joints = SUBSYSTEM_JOINTS[self.id]
        self.joint_count = len(self.joints)

        self.motors = [Motor(motor_id) for motor_id in SUBSYSTEM_MOTORS[self.id]]

        # For the singleton pattern
        self.initialized = True

    @abstractmethod
    def joints2motors(self, joint_angles):
        """
        Map the angles of the joints to the angles of the motors
        Each subclass should uniquely implement this method

        params
            joint_angles: a list of joint angles

        returns
            a list of motor angles
        """
        pass

    def write(self, joint_angles):
        """
        Map joint angles to motor angles and write to the motors

        params
            joint_angles: a list of joint angles
        """

        # Allow for single-joint subsystems to be controlled with a single float
        if type(joint_angles) == float:
            joint_angles = [joint_angles]

        motor_angles = self.joints2motors(joint_angles)

        for motor, angle in zip(self.motors, motor_angles): # type: ignore
            motor.write(angle)

    def read(self):
        """
        returns
            The positions of all joints in the subsystem
        """
        return [joint.read() for joint in self.joints]

    def at_position(self):
        """
        returns
            True if all motors in the subsystem are at their target angles
        """
        return all([motor.at_angle() for motor in self.motors])

    def __str__(self):
        formatted_values = [f"{value:.2f}" for value in self.read()]
        return self.id.name + ": " + str(formatted_values)

    def __getitem__(self, joint_index):
        """
        To support 'obj[joint_index]' for getting joints
        """
        return self.joints[joint_index]

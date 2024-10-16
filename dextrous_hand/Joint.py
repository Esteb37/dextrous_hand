#!/usr/bin/env python3

import dextrous_hand.ids as ids
from dextrous_hand.Motor import Motor
from abc import ABC, abstractmethod

class Joint(ABC):
    """
    A joint is a motor or a collection of motors that move together to achieve a specific angle.
    It is an abstract class so that each joint defines how to map motor angles to joint angles.
    """
    # For the singleton pattern
    _instances = {}

    def __new__(cls, name, *args, **kwargs):
        """
        Singleton pattern. Make sure only one instance of each Joint is created.
        If a joint with the same name has already been created, return that instance.
        """
        if name not in cls._instances:
            cls._instances[name] = super(Joint, cls).__new__(cls)
        return cls._instances[name]

    def __init__(self, id : ids.JOINTS):
        """
        params
            id [JOINTS]: the joint's id

        raises
            Exception: if the joint has no motors
        """
        # Avoid reinitialization if already initialized
        if hasattr(self, 'initialized') and self.initialized:
            return

        self.id = id

        from dextrous_hand.architecture import JOINT_MOTORS

        # Check if the joint has motors in the constants.py file
        if self.id not in JOINT_MOTORS or len(JOINT_MOTORS[self.id]) == 0:
            raise Exception("Joint " + self.id.name + " has no motors")

        self.motors = [Motor(motor_id) for motor_id in JOINT_MOTORS[self.id]]

        self.initialized = True

    @abstractmethod
    def motors2joint(self, motor_angles):
        """
        Map the angles of the motor(s) to the angle of the joint
        Each subclass should uniquely implement this method

        params
            motor_angles: a list of motor angles

        returns
            the angle of the joint
        """
        pass

    def read(self):
        """
        returns:
            The joint's current angle in radians
        """
        motor_angles = [motor.read() for motor in self.motors]
        return self.motors2joint(motor_angles)

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
        return self.id.name + ": " + f"{self.read():.2f} rad"

    def __getitem__(self, motor_index):
        """
        To support 'obj[motor_index]' for getting motors
        """
        return self.motors[motor_index]

#!/usr/bin/env python3

import dextrous_hand.ids as ids
from dextrous_hand.Motor import Motor
from abc import ABC, abstractmethod

from dextrous_hand.constants import SPOOL_RADIUS, IS_SIMULATION

import math
import numpy as np

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

        from dextrous_hand.joints_geometry import JOINTS_GEOMETRY

         # Check if the joint has geometry in the joints_geometry.py file
        if self.id not in JOINTS_GEOMETRY or len(JOINTS_GEOMETRY[self.id]) == 0:
            raise Exception("Joint " + self.id.name + " has no geometry")

        self.geometry = JOINTS_GEOMETRY[self.id]

        self.target = 0.0

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

    def joint2length(self, joint):
        """
        Map the joint angle to the lenght of the associated pair of tendons
        Each subclass should uniquely implement this method

        params
            joint: the angle of the joint

        returns
            a tuple containing the length of the inner and outer tendon
        """

        """
        TODO: Make this method general to the case of pin joints
        """
        if "radius" in self.geometry:
            a = self.geometry["radius"]/SPOOL_RADIUS
            return a*joint

        def rot_mat_z(tetha):
            C_z = np.array([[math.cos(tetha), -math.sin(tetha), 0],[math.sin(tetha), math.cos(tetha), 0],[0,0,1]])
            return C_z

        C_I1 = rot_mat_z(joint/2)
        C_I2 = rot_mat_z(joint)

        centers_vect = np.array([0,self.geometry["centers_distance"],0])
        p1 = self.geometry["T1"]
        p2 = self.geometry["T2"]
        p3 = self.geometry["T3"]
        p4 = self.geometry["T4"]

        tendon_12 = C_I1@centers_vect+C_I2@p2-p1
        length_12 = self.geometry["length_12_0"] - np.linalg.norm(tendon_12)

        tendon_34 = C_I1@centers_vect+C_I2@p4-p3
        length_34 = self.geometry["length_34_0"] - np.linalg.norm(tendon_34)

        return (length_12, length_34)

    def read(self):
        """
        returns:
            The joint's current angle in radians
        """
        if IS_SIMULATION:
            return self.target

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
        return self.id.name + ": " + f"{self.read():.3f} rad"

    def __getitem__(self, motor_index):
        """
        To support 'obj[motor_index]' for getting motors
        """
        return self.motors[motor_index]


class VirtualJoint():
    """
    A virtual joint is a joint that is not self-actuated
    """
    # For the singleton pattern
    _instances = {}

    def __new__(cls, name, *args, **kwargs):
        """
        Singleton pattern. Make sure only one instance of each Joint is created.
        If a joint with the same name has already been created, return that instance.
        """
        if name not in cls._instances:
            cls._instances[name] = super(VirtualJoint, cls).__new__(cls)
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
        self.initialized = True
        self.value = 0

    def read(self):
        """
        returns:
            The joint's current angle in radians
        """
        return self.value

    def write(self, value):
        """
        Set the joint's value
        """
        self.value = value

    def __str__(self):
        return self.id.name + ": " + f"{self.read():.3f}"
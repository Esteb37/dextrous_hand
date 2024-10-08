#!/usr/bin/env python3

from collections.abc import Iterable
from dextrous_hand.Joint import Joint
import dextrous_hand.constants as constants

class Subsystem():
    # For the singleton pattern
    _instances = {}

    def __new__(cls, subsystem_id : constants.IDS, *args, **kwargs):
        """
        Singleton pattern. Make sure only one instance of each Subsystem is created.
        If it has already been created, return the existing instance.

        param subsystem_id: the ID of the subsystem
        type subsystem_id: constants.IDS
        """
        if subsystem_id not in cls._instances:
            cls._instances[subsystem_id] = super(Subsystem, cls).__new__(cls)
        return cls._instances[subsystem_id]

    def __init__(self, subsystem_id : constants.IDS):
        # Avoid reinitialization if the instance already exists
        if hasattr(self, 'initialized') and self.initialized:
            return

        self.id = subsystem_id
        self.name = self.id.name
        self.joint_count = len(constants.MOTOR_PORTS[self.id])

        if self.joint_count == 0:
            raise Exception("Subsystem ", self.id.name, " has no joints")

        if self.joint_count != len(constants.MOTOR_LIMITS[self.id]):
            raise Exception("Subsystem ", self.id.name, " has ", self.joint_count, " joints, but ", len(constants.MOTOR_LIMITS[self.id]), " limits were provided")

        # Create a Joint instance for each joint in the subsystem
        self.joints = [Joint(self.id.name + "." + str(i),
                             constants.MOTOR_PORTS[self.id][i],
                             constants.MOTOR_LIMITS[self.id][i])
                             for i in range(self.joint_count)]

        # For the singleton pattern
        self.initialized = True

    def set_positions(self, positions : list[float] | int):
        """
        Set the positions of all joints in the subsystem simultaneously

        param positions: list of positions for each joint in the subsystem
        """
        if type(positions) is int:
            if self.joint_count != 1:
                raise Exception("Positions must be a list of ", self.joint_count, " elements. Received a single value")
            self.joints[0].set_position(positions)

        elif isinstance(positions, Iterable):
            if len(positions) != self.joint_count:
                raise Exception("Positions array must have ", self.joint_count, " elements. Received ", len(positions), " elements")

            for i in range(self.joint_count):
                self.joints[i].set_position(positions[i])


    def print(self, verbose : bool = False):
        for i in range(self.joint_count):
            self.joints[i].print(verbose)

    def __str__(self):
        string = self.id.name + "\n"
        string += "[" + ", ".join([str(joint.position) for joint in self.joints]) + "]\n"
        return string

    # To support 'obj[joint_index]' for getting joints
    def __getitem__(self, joint_index):
        return self.joints[joint_index]

    @property
    def positions(self):
        return [joint.position for joint in self.joints]

    @positions.setter
    def positions(self, positions):
        self.set_positions(positions)

    @property
    def position(self):
        if self.joint_count != 1:
            raise Exception("Subsystem ", self.id.name, " has ", self.joint_count, " joints. Use 'positions' instead")
        return self.joints[0].position

    @position.setter
    def position(self, position):
        if self.joint_count != 1:
            raise Exception("Subsystem ", self.id.name, " has ", self.joint_count, " joints. Use 'positions' instead")
        self.joints[0].position = position
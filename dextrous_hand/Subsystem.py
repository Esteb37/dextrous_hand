#!/usr/bin/env python3

from collections.abc import Iterable
from dextrous_hand.Joint import Joint
import dextrous_hand.constants as constants

class Subsystem():
    # For the singleton pattern
    _instances = {}

    def __new__(cls, subsystem_id : constants.SUBSYSTEMS, *args, **kwargs):
        """
        Singleton pattern. Make sure only one instance of each Subsystem is created.
        If it has already been created, return the existing instance.

        param subsystem_id: the ID of the subsystem
        type subsystem_id: constants.IDS
        """
        if subsystem_id not in cls._instances:
            cls._instances[subsystem_id] = super(Subsystem, cls).__new__(cls)
        return cls._instances[subsystem_id]

    def __init__(self, subsystem_id : constants.SUBSYSTEMS):
        # Avoid reinitialization if the instance already exists
        if hasattr(self, 'initialized') and self.initialized:
            return

        self.id = subsystem_id
        self.joint_count = len(constants.SUBSYSTEM_JOINTS[self.id])

        if self.joint_count == 0:
            raise Exception("Subsystem ", self.id.name, " has no joints")

        # Create a Joint instance for each joint in the subsystem
        self.joints = [Joint(joint_id) for joint_id in constants.SUBSYSTEM_JOINTS[self.id]]

        # For the singleton pattern
        self.initialized = True

    def write(self, positions : list[float] | int):
        """
        Set the positions of all joints in the subsystem simultaneously

        param positions: list of positions for each joint in the subsystem
        """
        if type(positions) is int:
            if self.joint_count != 1:
                raise Exception("Positions must be a list of ", self.joint_count, " elements. Received a single value")
            self.joints[0].write(positions)

        elif isinstance(positions, Iterable):
            if len(positions) != self.joint_count:
                raise Exception("Positions array must have ", self.joint_count, " elements. Received ", len(positions), " elements")

            for i in range(self.joint_count):
                self.joints[i].write(positions[i])

        return self.at_position()

    def read(self):
        """
        Read the positions of all joints in the subsystem
        """
        return [joint.read() for joint in self.joints]

    def at_position(self):
        """
        Check if all joints in the subsystem are at their target positions
        """
        return all([joint.at_position() for joint in self.joints])

    def __str__(self):
        formatted_values = [f"{value:.3f}" for value in self.read()]
        return self.id.name + ": " + str(formatted_values)

    # To support 'obj[joint_index]' for getting joints
    def __getitem__(self, joint_index):
        return self.joints[joint_index]

#!/usr/bin/env python3

from collections.abc import Iterable
from dextrous_hand.Joint import Joint
import dextrous_hand.constants as constants

class Subsystem():
    """
    A subsystem is any collection of joints that can be controlled together.
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

    def __init__(self, subsystem_id : constants.SUBSYSTEMS):
        """
        params
            subsystem_id [SUBSYSTEMS]: the subsystem's id
        """
        # Avoid reinitialization if the instance already exists
        if hasattr(self, 'initialized') and self.initialized:
            return

        self.id = subsystem_id

        # Check if the subsystem has joints in the constants.py file
        if self.id not in constants.SUBSYSTEM_JOINTS or len(constants.SUBSYSTEM_JOINTS[self.id]) == 0:
            raise Exception("Subsystem " + self.id.name + " has no joints")

        self.joint_count = len(constants.SUBSYSTEM_JOINTS[self.id])

        # Create a Joint instance for each joint in the subsystem
        self.joints = [Joint(joint_id) for joint_id in constants.SUBSYSTEM_JOINTS[self.id]]

        # For the singleton pattern
        self.initialized = True

    def write(self, positions : list[float] | int):
        """
        Set the positions of all joints in the subsystem simultaneously

        params
            positions: list of positions for each joint in the subsystem, or a single position for a single-joint subsystem

        returns
            True if all joints are at their target positions

        raises
            Exception: if the positions array has the wrong number of elements
        """

        # If positions is a single value, write it to the single joint
        if type(positions) is int:
            if self.joint_count != 1:
                raise Exception("Positions must be a list of " + str(self.joint_count) + " elements. Received a single value")
            self.joints[0].write(positions)

        elif isinstance(positions, Iterable):
            if len(positions) != self.joint_count:
                raise Exception("Positions array must have " + str(self.joint_count) + " elements. Received " + str(len(positions)) + " elements")

            for i in range(self.joint_count):
                self.joints[i].write(positions[i])

        return self.at_position()

    def read(self):
        """
        returns
            The positions of all joints in the subsystem
        """
        return [joint.read() for joint in self.joints]

    def at_position(self):
        """
        returns
            True if all joints in the subsystem are at their target positions
        """
        return all([joint.at_position() for joint in self.joints])

    def __str__(self):
        formatted_values = [f"{value:.2f}" for value in self.read()]
        return self.id.name + ": " + str(formatted_values)

    def __getitem__(self, joint_index):
        """
        To support 'obj[joint_index]' for getting joints
        """
        return self.joints[joint_index]

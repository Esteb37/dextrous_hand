#!/usr/bin/env python3

from dextrous_hand.Joint import Joint
import dextrous_hand.constants as constants

class Finger():
    # For the singleton pattern
    _instances = {}

    def __new__(cls, finger_id : constants.IDS, *args, **kwargs):
        """
        Singleton pattern. Make sure only one instance of each Finger is created.
        If it has already been created, return the existing instance.

        param finger_id: the ID of the finger
        type finger_id: constants.IDS
        """
        if finger_id not in cls._instances:
            cls._instances[finger_id] = super(Finger, cls).__new__(cls)
        return cls._instances[finger_id]

    def __init__(self, finger_id : constants.IDS):
        # Avoid reinitialization if the instance already exists
        if hasattr(self, 'initialized') and self.initialized:
            return

        self.id = finger_id
        self.joint_count = len(constants.MOTOR_PORTS[self.id])

        # Create a Joint instance for each joint in the finger
        self.joints = [Joint(self.id.name + "." + str(i),
                             constants.MOTOR_PORTS[self.id][i],
                             constants.MOTOR_LIMITS[self.id][i])
                             for i in range(self.joint_count)]

        # For the singleton pattern
        self.initialized = True

    def set_positions(self, positions : list[float]):
        """
        Set the positions of all joints in the finger simultaneously

        param positions: list of positions for each joint in the finger
        """
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


THUMB = Finger(constants.IDS.THUMB)
INDEX = Finger(constants.IDS.INDEX)
MIDDLE = Finger(constants.IDS.MIDDLE)
RING = Finger(constants.IDS.RING)
PINKY = Finger(constants.IDS.PINKY)

FINGERS = [THUMB, INDEX, MIDDLE, RING, PINKY]

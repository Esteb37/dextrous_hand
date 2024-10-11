#!/usr/bin/env python3

from dextrous_hand.Finger import FINGERS
from dextrous_hand.Abduction import ABDUCTION
from dextrous_hand.Wrist import WRIST
from dextrous_hand.utils import finger_pos_to_matrix
from dextrous_hand.Subsystem import Subsystem
import dextrous_hand.constants as constants
from dextrous_hand.MotorBridge import MotorBridge

class Hand():
    _instance = None

    def __new__(cls, *args, **kwargs):
        """
        Singleton pattern. Make sure only one instance of Hand is created.
        If it has already been created, return the existing instance
        """
        if cls._instance is None:
            cls._instance = super(Hand, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if hasattr(self, 'initialized') and self.initialized:
            return

        self.motor_bridge = MotorBridge()

        self.initialized = True

    def set_fingers(self, positions):
        """
        Set the positions of all fingers simultaneously.

        params
            positions: a matrix or dictionary of finger positions
                matrix:

                        [float, float, float, <- Finger 1
                        float, float, float,
                        ...
                        float, float, float]

                dictionary:

                        {THUMB: [float, float, float],
                        INDEX: [float, float, float],
                        ...
                        PINKY: [float, float, float]}

        returns
            True if all fingers are at their target positions

        raises
            Exception: if the positions array has the wrong number of elements
        """

        # If positions is a dictionary, convert it to a matrix
        if type(positions) == dict:
            positions = finger_pos_to_matrix(positions)

        # Write the positions to each finger
        for i, finger in enumerate(FINGERS):
            finger.write(positions[i * 3 : (i + 1) * 3])

        # Check if all fingers are at position
        at_position = True
        for finger in FINGERS:
            at_position = at_position and finger.at_position()
        return at_position

    def set_abduction(self, position):
        return ABDUCTION.write(position)

    def set_wrist(self, position):
        return WRIST.write(position)

    def update_motor_positions(self):
        """
        Read the encoder positions of all motors
        """
        self.motor_bridge.update_positions()

    def write_motor_targets(self):
        """
        Write the target positions to all motors
        """
        self.motor_bridge.write_targets()

    def __str__(self):
        string = ""
        for subsystem_id in constants.SUBSYSTEMS:
            subsystem = Subsystem(subsystem_id)
            string += str(subsystem) + "\n"
            for joint in subsystem.joints:
                string += "\t" + str(joint) + "\n"
                for motor in joint.motors:
                    string += "\t\t" + str(motor) + "\n"
        return string

HAND = Hand()
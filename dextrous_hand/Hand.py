#!/usr/bin/env python3

from dextrous_hand.Finger import FINGERS
from dextrous_hand.Wrist import WRIST
from dextrous_hand.utils import finger_pos_to_matrix

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

        self.initialized = True

    def set_fingers(self, positions : list[list[float]]):
        """
        Set the positions of all fingers simultaneously.

        NOTE: The dimension of the position array should be 3x1 because the DIP
              angle cannot be set.

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

        assert len(positions) == len(FINGERS)
        assert len(positions[0]) == 3

        # Write the positions to each finger
        for i, finger in enumerate(FINGERS):
            finger.write(positions[i])

        # Check if all fingers are at position
        at_position = True
        for finger in FINGERS:
            at_position = at_position and finger.at_position()
        return at_position

    def set_wrist(self, position):
        return WRIST.write(position)

    def get_fingers(self):

        # Get the positions of all fingers
        positions = []
        for finger in FINGERS:
            # only the first 3 columns
            positions.append(finger.read())

        return positions

    def get_wrist(self):
        return float(WRIST.read()[0])

    def __str__(self):
        string = ""
        for subsystem in FINGERS + [WRIST]:
            string += str(subsystem) + "\n"
            for joint in subsystem.joints:
                string += "\t" + str(joint) + "\n"
                for motor in joint.motors:
                    string += "\t\t" + str(motor) + "\n"
        return string

# Singleton instance
HAND = Hand()
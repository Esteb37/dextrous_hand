#!/usr/bin/env python3

from dextrous_hand.Finger import FINGERS
from dextrous_hand.Abduction import ABDUCTION
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

    def set_fingers(self, positions):
        """
        Set the positions of all fingers simultaneously.

        param positions: a matrix or dictionary of finger positions
        """

        # If positions is a dictionary, convert it to a matrix
        if type(positions) == dict:
            positions = finger_pos_to_matrix(positions)

        for i, finger in enumerate(FINGERS):
            finger.positions = positions[i * 3 : (i + 1) * 3]

    def set_abduction(self, position):
        ABDUCTION.position = position

    def set_wrist(self, position):
        WRIST.position = position

    def print(self, verbose = False):
        for finger in FINGERS:
            finger.print(verbose)
        WRIST.print(verbose)
        ABDUCTION.print(verbose)
        print()

    def __str__(self):
        string = "---- Hand ----\n"
        for finger in FINGERS:
            string += finger.id.name+": "+str(finger.positions)+"\n"
        string += WRIST.name + ": " + str(WRIST.position) + "\n"
        string += ABDUCTION.name + ": " + str(ABDUCTION.position) + "\n"
        return string

HAND = Hand()
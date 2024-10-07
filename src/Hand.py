#!/usr/bin/env python3

from Finger import FINGERS
from Joint import ABDUCTION, WRIST

def matrix_to_fingers_pos(matrix):
    """
    Turns a position matrix into human-readable dictionary

    param matrix: a matrix of finger positions, of shape

                    [float, float, float, <- Finger 1
                     float, float, float,
                     ...
                     float, float, float]

    return positions: a dictionary of finger positions, of shape

                    {THUMB: [float, float, float],
                     INDEX: [float, float, float],
                     ...
                     PINKY: [float, float, float]}
    """

    if len(matrix) != len(FINGERS) or len(matrix[0]) != FINGERS[0].joint_count:
        raise Exception("Matrix must have ", len(FINGERS),
                        " rows and ", FINGERS[0].joint_count, " columns. Received ",
                        len(matrix), " rows and ", len(matrix[0]), " columns")

    positions = {}
    for i, finger in enumerate(FINGERS):
        positions[finger] = matrix[i]

    return positions

def finger_pos_to_matrix(positions):
    """
    Turns a human-readable position dict into position matrix

    param positions: a dictionary of finger positions, of shape
                    {THUMB: [float, float, float],
                    INDEX: [float, float, float],
                    ...
                    PINKY: [float, float, float]}

    return matrix: a matrix of finger positions, of shape
                    [float, float, float,
                    float, float, float,
                    ...
                    float, float, float]
    """
    matrix = []
    for finger in FINGERS:
        matrix.append(positions[finger.id])

    return matrix

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
            finger.set_positions(positions[i])

    def set_abduction(self, position):
        ABDUCTION.set_position(position)

    def set_wrist(self, position):
        WRIST.set_position(position)

    def print(self, verbose = False):
        for finger in FINGERS:
            finger.print(verbose)
        WRIST.print(verbose)
        ABDUCTION.print(verbose)
        print()

    def __str__(self):
        string = "---- Hand ----\n"
        for finger in FINGERS:
            string += finger.id.name+": [" + ", ".join([str(joint.position) for joint in finger.joints]) + "]\n"
        string += WRIST.name + ": " + str(WRIST.position) + "\n"
        string += ABDUCTION.name + ": " + str(ABDUCTION.position) + "\n"
        return string

HAND = Hand()
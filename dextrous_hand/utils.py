#!/usr/bin/env python3

from dextrous_hand.Finger import FINGERS
from std_msgs.msg import Float32MultiArray

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
        raise Exception("Matrix must have " + str(len(FINGERS)) +
                        " rows and " + str(FINGERS[0].joint_count)  +
                        " columns. Received " + str(len(matrix)) + " rows and "
                        + str(len(matrix[0])) + " columns")

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

def matrix_to_message(matrix):
    """
    Takes a matrix and converts it to a Float32MultiArray message
    """
    msg = Float32MultiArray()
    data = list(matrix.flatten())
    msg.data = data
    return msg
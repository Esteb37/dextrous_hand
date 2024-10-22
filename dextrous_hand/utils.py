#!/usr/bin/env python3

from dextrous_hand.Finger import FINGERS
from std_msgs.msg import Float32MultiArray
import numpy as np

def matrix_to_fingers_pos(matrix):
    """
    Turns a position matrix into human-readable dictionary

    params
        a matrix of finger positions, of shape

                    [float, float, float, <- Finger 1
                     float, float, float,
                     ...
                     float, float, float]

    returns
        a dictionary of finger positions, of shape

                    {THUMB: [float, float, float],
                     INDEX: [float, float, float],
                     ...
                     PINKY: [float, float, float]}

    raises
        Exception: if the matrix has the wrong number of rows or columns
    """

    if len(matrix) != len(FINGERS) or len(matrix[0]) != 3:
        raise Exception("Matrix must have " + str(len(FINGERS)) +
                        " rows and " + str(3)  +
                        " columns. Received " + str(len(matrix)) + " rows and "
                        + str(len(matrix[0])) + " columns")

    positions = {}
    for i, finger in enumerate(FINGERS):
        positions[finger] = matrix[i]

    return positions

def finger_pos_to_matrix(positions):
    """
    Turns a human-readable position dict into position matrix

    params
        positions: a dictionary of finger positions, of shape
                    {
                     PINKY: [float, float, float]
                     RING: [float, float, float],
                     ...
                     THUMB: [float, float, float],

                    }

    returns
        a matrix of finger positions, of shape
                    [
                     [float, float, float],
                     [float, float, float],
                     ...
                     [float, float, float]
                    ]
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
    data = list(np.array(matrix).flatten())
    msg.data = data
    return msg

def message_to_matrix(message : Float32MultiArray, shape):
    """
    Takes a Float32MultiArray message and converts it to a matrix
    """
    return np.array(message.data).reshape(shape).tolist()
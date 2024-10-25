#!/usr/bin/env python3

from std_msgs.msg import Float32MultiArray
import numpy as np

def matrix_to_message(matrix):
    """
    Takes a matrix and converts it to a Float32MultiArray message
    """
    msg = Float32MultiArray()

    # Make sure all arrays have the same length
    max_len = max([len(row) for row in matrix])
    for row in matrix:
        row += [0] * (max_len - len(row))

    data = list(np.array(matrix, dtype = float).flatten())
    msg.data = data
    return msg

def message_to_matrix(message : Float32MultiArray, num_rows):
    """
    Takes a Float32MultiArray message and converts it to a matrix
    """
    num_cols = len(message.data) // num_rows
    shape = (num_rows, num_cols)
    return np.array(message.data).reshape(shape).tolist()
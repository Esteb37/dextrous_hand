#!/usr/bin/env python3

import numpy as np
from pathlib import Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, Quaternion, Point


def matrix_to_message(matrix):
    """
    Takes a matrix and converts it to a Float32MultiArray message
    """
    msg = Float32MultiArray()

    square_matrix = []

    # Make sure all arrays have the same length
    max_len = max([len(row) for row in matrix])
    for row in matrix:
        new_row = row + [0.0] * (max_len - len(row))
        square_matrix.append(new_row)

    data = list(np.array(square_matrix, dtype = float).flatten())
    msg.data = data
    return msg

def message_to_matrix(message : Float32MultiArray, num_rows):
    """
    Takes a Float32MultiArray message and converts it to a matrix
    """
    num_cols = len(message.data) // num_rows
    shape = (num_rows, num_cols)
    return np.array(message.data).reshape(shape).tolist()

def as_point(list):
    """
    Takes a list and converts it to a Point message
    """
    return Point(x = list[0], y = list[1], z = list[2])

def as_quaternion(list):
    """
    Takes a list and converts it to a Quaternion message
    """
    return Quaternion(x = list[0], y = list[1], z = list[2], w = list[3])

def from_point(point):
    """
    Takes a Point message and converts it to a list
    """
    return [point.x, point.y, point.z]

def from_quaternion(quaternion):
    """
    Takes a Quaternion message and converts it to a list
    """
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

def pos_orient_to_pose(pos : list[float], orient : list[float]):
    """
    Takes a position and orientation and converts it to a Pose message
    """
    pose = PoseStamped()
    pose.pose.position = as_point(pos)
    pose.pose.orientation = as_quaternion(orient)

    return pose

def pose_to_pos_orient(pose : PoseStamped) -> tuple[list[float], list[float]]:
    """
    Takes a Pose message and converts it to a position and orientation
    """
    pos = from_point(pose.pose.position)
    orient = from_quaternion(pose.pose.orientation)
    return pos, orient


def parent_dir():
    """
    Gets the directory where the node is running from
    """

    current_dir = Path(__file__).resolve().parent

    # Traverse upward until "install" directory is found
    install_dir = None
    for parent in current_dir.parents:
        if parent.name == "install":
            install_dir = parent
            break

    # If "install" directory is not found, traverse upward until "src" directory is found
    if install_dir is None:
        for parent in current_dir.parents:
            if parent.name == "src":
                install_dir = parent
                break

    if install_dir is None:
        raise FileNotFoundError("Could not find the install or src directory.")

    parent_dir = install_dir.parent
    return (parent_dir / "src" / "dextrous_hand").as_posix()

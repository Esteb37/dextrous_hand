#!/usr/bin/env python3

from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from pathlib import Path

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

def pos_orient_to_pose(pos : list[float], orient : list[float]):
    """
    Takes a position and orientation and converts it to a Pose message
    """
    pose = PoseStamped()
    pose.pose.position = Point(x = pos[0], y = pos[1], z = pos[2])
    orientation = R.from_euler('xyz', orient, degrees = True).as_quat()

    pose.pose.orientation = Quaternion(x = orientation[0], y = orientation[1], z = orientation[2], w = orientation[3])

    return pose

def pose_to_pos_orient(pose : PoseStamped) -> tuple[list[float], list[float]]:
    """
    Takes a Pose message and converts it to a position and orientation
    """
    pos = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
    orientation = R.from_quat([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]).as_euler('xyz', degrees = True).tolist()

    return pos, orientation


def parent_dir():
    # Go back until you find the "install" folder
    current_dir = Path(__file__).resolve().parent

    # Traverse upward until "install" directory is found
    install_dir = None
    for parent in current_dir.parents:
        if parent.name == "install":
            install_dir = parent
            break

    if install_dir is None:
        for parent in current_dir.parents:
            if parent.name == "src":
                install_dir = parent
                break

    if install_dir is None:
        raise FileNotFoundError("Could not find the install or src directory.")

    parent_dir = install_dir.parent
    return (parent_dir / "src" / "dextrous_hand").as_posix()
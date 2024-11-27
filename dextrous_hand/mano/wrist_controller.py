import numpy as np
from scipy.spatial.transform import Rotation

class WristController:
    def __init__(self):
        pass

    def get_wrist_command(self, wrist_pos: list[float], wrist_quat: list[float], elbow_quat: list[float]) -> float:
        """
        Get the commands for the Franka and the Hand based on the user input and the arm pose cmd
        """
        # Get the wrist pose in the elbow frame, so that we extract the rotation
        # (The translation between elbow and wrist is 0)
        wrist_euler_in_elbow_frame = (Rotation.from_quat(wrist_quat) * Rotation.from_quat(elbow_quat).inv()).as_euler('xyz', degrees=False)
        wrist_joint = wrist_euler_in_elbow_frame[0]
        return wrist_joint

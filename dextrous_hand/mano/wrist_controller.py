import numpy as np
from scipy.spatial.transform import Rotation

class WristController:
    def __init__(self):
        pass

    def get_wrist_command(self, wrist_init_quat: list[float], wrist_quat: list[float], elbow_quat: list[float]):
        """
        Get the commands for the Franka and the Hand based on the user input and the arm pose cmd
        """

        wrist_rotation = Rotation.from_quat(wrist_quat)
        elbow_rotation = Rotation.from_quat(elbow_quat)
        
        # Compute relative rotation from elbow to wrist
        relative_rotation = elbow_rotation.inv() * wrist_rotation
        relative_quat = relative_rotation.as_quat().tolist()
        # Extract the rotation matrix
        relative_matrix = relative_rotation.as_matrix()
        
        # Extract the angle of rotation about the x-axis
        flexion_angle = -np.arctan2(relative_matrix[2, 1], relative_matrix[2, 2])
        
        return flexion_angle, relative_quat

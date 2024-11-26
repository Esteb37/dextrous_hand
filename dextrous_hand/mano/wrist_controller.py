import numpy as np
from scipy.spatial.transform import Rotation

class WristController:
    def __init__(self):
        pass

    def get_commands(self, arm_pos: list[float], arm_quat: list[float]) -> tuple[list[float], list[float], float]:
        """
        Get the commands for the Franka and the Hand based on the user input and the arm pose cmd
        """
        # The first goal is to isolate the rotation that the wrist of our hand can do.
        # The rotation of the wrist should be around the x axis in the input
        arm_rot = Rotation.from_quat(arm_quat)
        arm_euler = arm_rot.as_euler('xyz')

        # Isolate the rotation of the wrist
        wrist_joint = arm_euler[0]

        # # Remove that rotation from the arm
        arm_euler[1] = 0

        arm_pos_cmd = arm_pos
        arm_quat_cmd = Rotation.from_euler('xyz', arm_euler).as_quat().tolist()
        arm_quat_cmd = arm_quat_cmd / np.linalg.norm(arm_quat_cmd)


        # arm_quat_cmd = np.array(arm_quat)
        # arm_quat_cmd = (arm_quat_cmd / np.linalg.norm(arm_quat_cmd)).tolist()
        # arm_pos_cmd = arm_pos
        # print("Wrist joint: ", wrist_joint)
        # print("Arm pos cmd: ", arm_pos_cmd)
        # print("Arm quat cmd: ", arm_quat_cmd)       


        return arm_pos_cmd, arm_quat_cmd, wrist_joint

#!/usr/bin/env python3

from dextrous_hand.constants import FRANKA_CENTER_ORIENTATION, FRANKA_CENTER_POSITION, IS_SIMULATION
from dextrous_hand.utils import pos_orient_to_pose, pose_to_pos_orient
from scipy.spatial.transform import Rotation as R

class Arm:

    AT_TARGET_THRESHOLD = 0.01

    __instance = None

    def __new__(cls, *args, **kwargs):
        """
        Singleton pattern. Make sure only one instance of the arm is created.
        If it has already been created, return the existing instance.
        """
        if cls.__instance is None:
            cls.__instance = super(Arm, cls).__new__(cls)
        return cls.__instance

    def __init__(self):
        if hasattr(self, 'initialized') and self.initialized:
            return

        self._target = pos_orient_to_pose(FRANKA_CENTER_POSITION, FRANKA_CENTER_ORIENTATION)
        self._pose = pos_orient_to_pose(FRANKA_CENTER_POSITION, FRANKA_CENTER_ORIENTATION)

        self.initialized = True

    def offset_pose(self, pose, positive = True):
        position = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        orientation = pose.pose.orientation
        orientation_rad = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_euler('xyz', degrees = True).tolist()

        if positive:
            for i in range(3):
                position[i] += FRANKA_CENTER_POSITION[i]
                orientation_rad[i] += FRANKA_CENTER_ORIENTATION[i]

        else:
            for i in range(3):
                position[i] -= FRANKA_CENTER_POSITION[i]
                orientation_rad[i] -= FRANKA_CENTER_ORIENTATION[i]

        new_pose = pos_orient_to_pose(position, orientation_rad)

        return new_pose

    def write(self, position, orientation):
        self._target = pos_orient_to_pose(position, orientation)
        self._target = self.offset_pose(self._target, positive = True)

    @property
    def target(self):
        return pose_to_pos_orient(self.offset_pose(self._target, positive = False))

    @property
    def pose(self):
        return pose_to_pos_orient(self.offset_pose(self._pose, positive = False))

    @property
    def position(self):
        return self.pose[0]

    @property
    def orientation(self):
        return self.pose[1]

    @property
    def target_position(self):
        return self.target[0]

    @property
    def target_orientation(self):
        return self.target[1]

    def target_msg(self):
        return self._target

    def update(self, current_pose):
        if IS_SIMULATION:
            self._pose = self._target
        else:
            self._pose = current_pose

    def at_target(self):
        return self._at_target_position() and self._at_target_orientation()

    def _at_target_position(self):
        return self._distance(self.position, self.target_position) < self.AT_TARGET_THRESHOLD

    def _at_target_orientation(self):
        return self._distance(self.orientation, self.target_orientation) < self.AT_TARGET_THRESHOLD

    def _distance(self, a, b):
        return sum((a_i - b_i) ** 2 for a_i, b_i in zip(a, b)) ** 0.5

    def __str__(self):
        formatted_pos = [f"{value:.3f}" for value in self.position]
        formatted_or = [f"{value:.3f}" for value in self.orientation]
        return "ARM:\t" + str(formatted_pos) + "\n\t" + str(formatted_or)


ARM = Arm()
#!/usr/bin/env python3

from scipy.spatial.transform import Rotation as R

from dextrous_hand.utils.utils import pos_orient_to_pose, pose_to_pos_orient, from_quaternion, from_point
from dextrous_hand.utils.constants import ARM_CONSTANTS, GLOBAL_CONSTANTS

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

        self._center = pos_orient_to_pose(ARM_CONSTANTS["CENTER_POSITION"],
                                         ARM_CONSTANTS["CENTER_ORIENTATION"])

        self._target = pos_orient_to_pose(ARM_CONSTANTS["CENTER_POSITION"],
                                         ARM_CONSTANTS["CENTER_ORIENTATION"])

        self._pose = pos_orient_to_pose(ARM_CONSTANTS["CENTER_POSITION"],
                                        ARM_CONSTANTS["CENTER_ORIENTATION"])

        self._range_x = ARM_CONSTANTS["RANGE_X"]
        self._range_y = ARM_CONSTANTS["RANGE_Y"]
        self._range_z = ARM_CONSTANTS["RANGE_Z"]

        self.initialized = True

    def offset_pose(self, pose, positive):
        """
        Get the real position of the arm by adding the center offset
        """
        position = from_point(pose.pose.position)
        orientation = from_quaternion(pose.pose.orientation)

        center_position = from_point(self._center.pose.position)
        center_orientation = from_quaternion(self._center.pose.orientation)

        for i in range(3):
            if positive:
                position[i] += center_position[i]
            else:
                position[i] -= center_position[i]

        if positive:
            orientation = R.from_quat(orientation)
            center_orientation = R.from_quat(center_orientation)
            r = center_orientation * orientation
            orientation = r.as_quat().tolist()
        else:
            orientation = R.from_quat(orientation)
            center_orientation = R.from_quat(center_orientation)
            r = center_orientation.inv() * orientation
            orientation = r.as_quat().tolist()


        return pos_orient_to_pose(position, orientation)

    def restrict_position(self, pose):
        """
        Restrict the position of the arm to the range of motion
        """
        position = from_point(pose.pose.position)
        position[0] = max(self._range_x[0], min(self._range_x[1], position[0]))
        position[1] = max(self._range_y[0], min(self._range_y[1], position[1]))
        position[2] = max(self._range_z[0], min(self._range_z[1], position[2]))

        return pos_orient_to_pose(position, from_quaternion(pose.pose.orientation))

    def write(self, position, orientation):
        self._target = pos_orient_to_pose(position, orientation)
        self._target = self.offset_pose(self._target, positive = True)
        self._target = self.restrict_position(self._target)

    @property
    def target(self):
        """
        Get a touple with the target position vector and orientation quaternion
        """
        return pose_to_pos_orient(self.offset_pose(self._target, positive = False))

    @property
    def pose(self):
        """
        Get a touple with the current position vector and orientation quaternion
        """
        return pose_to_pos_orient(self.offset_pose(self._pose, positive = False))

    @property
    def POSITION(self):
        return self.pose[0]

    @property
    def ORIENTATION(self):
        """
        As quaternion
        """
        return self.pose[1]

    @property
    def ORIENTATION_XYZ(self):
        """
        In euler radians
        """
        return R.from_quat(self.ORIENTATION).as_euler('xyz')
    @property
    def target_position(self):
        return self.target[0]

    @property
    def target_orientation(self):
        return self.target[1]

    def target_msg(self):
        return self._target

    def update(self, current_pose):
        if GLOBAL_CONSTANTS["IS_SIMULATION"]:
            self._pose = self._target
        else:
            self._pose = current_pose

    def at_target(self):
        return self._at_target_position() and self._at_target_orientation()

    def _at_target_position(self):
        current = from_point(self._pose.pose.position)
        target = from_point(self._target.pose.position)
        return self._distance(current, target) < self.AT_TARGET_THRESHOLD

    def _at_target_orientation(self):
        current = from_quaternion(self._pose.pose.orientation)
        target = from_quaternion(self._target.pose.orientation)
        return self._distance(current, target) < self.AT_TARGET_THRESHOLD

    def _distance(self, a, b):
        return sum([(a[i] - b[i])**2 for i in range(len(a))]) ** 0.5

    def __str__(self):
        formatted_pos = [f"{value:.3f}" for value in self.POSITION]
        formatted_or = [f"{value:.3f}" for value in self.ORIENTATION]
        formatted_xyz = [f"{value:.3f}" for value in self.ORIENTATION_XYZ]
        return "ARM:\t" + str(formatted_pos) + \
               "\n\tEul: " +  str(formatted_xyz) + \
               "\tQuat: " + str(formatted_or)


ARM = Arm()
#!/usr/bin/env python3

import numpy as np
from dataclasses import dataclass
from std_msgs.msg import Float32MultiArray
from scipy.spatial.transform import Rotation as R

from dextrous_hand.utils import ids
import dextrous_hand.utils.utils as utils
from dextrous_hand.utils.constants import CONFIGS

HandConfigIndex = int | str | ids.SUBSYSTEMS | ids.JOINTS
@dataclass
class HandConfig:
    """
    A dataclass to encapsulate the configuration of all the joints in the hand.

    Can be created like:
        HandConfig(PINKY = [0, 0, 0],
                    RING = [0, 0, 0],
                    MIDDLE = [0, 0, 0],
                    INDEX = [0, 0, 0],
                    THUMB = [0, 0, 0, 0],
                    WRIST = [0],
                    POSITION = [0, 0, 0],
                    ORIENTATION = [0, 0, 0, 1])
    """

    PINKY : list[float]
    RING : list[float]
    MIDDLE : list[float]
    INDEX : list[float]
    THUMB : list[float]
    WRIST : list[float]
    POSITION : list[float]
    ORIENTATION : list[float]

    def __init__(self, config_name = None, unrestricted = False, **kwargs):
        """
        Initialize the HandConfig object with the given values or zeros if not provided

        params
            PINKY = [float, float, float]
            RING = [float, float, float]
            MIDDLE = [float, float, float]
            INDEX = [float, float, float]
            THUMB = [float, float, float, float]
            WRIST = [float]
            POSITION = [float, float, float] # x, y, z
            ORIENTATION = [float, float, float, float] # x, y, z, w
        """
        self.PINKY = [0.0, 0.0, 0.0]
        self.RING = [0.0, 0.0, 0.0]
        self.MIDDLE = [0.0, 0.0, 0.0]
        self.INDEX = [0.0, 0.0, 0.0]
        self.THUMB = [0.0, 0.0, 0.0, 0.0]
        self.WRIST = [0.0]
        self.POSITION = [0.0, 0.0, 0.0]
        self.ORIENTATION = [0.0, 0.0, 0.0, 1.0]

        # For debug purposes, we sometimes want to remove the "triangle" restriction
        self.unrestricted = unrestricted

        # Create a map to easily access the joint values by joint ID
        from dextrous_hand.utils.architecture import SUBSYSTEM_JOINTS
        self.joint_map = {}

        for subsystem in ids.SUBSYSTEMS:
            for joint in SUBSYSTEM_JOINTS[subsystem]:
                joint_ids = [joint.id for joint in SUBSYSTEM_JOINTS[subsystem]]
                index = joint_ids.index(joint.id)
                self.joint_map[joint.id] = (subsystem, index)

        # If a config name is provided, load the values from the config
        if config_name is not None:
            if config_name not in CONFIGS:
                raise ValueError(f"Config {config_name} not found in CONFIGS")

            for key, value in CONFIGS[config_name].items():
                if type(value) is float:
                    value = [value]
                self[key] = value

        # Set the values provided in the kwargs
        for key, value in kwargs.items():
            if type(value) is float or type(value) is np.float64:
                value = [value]
            self[key] = value

        self.restrict()

    def restrict(self):
        """
        Implements the "triangle" restriction on the joint angles
        """
        if self.unrestricted:
            return

        # The include is here to avoid circular imports
        from dextrous_hand.subsystems.Finger import PINKY, RING, MIDDLE, INDEX, THUMB

        self.PINKY = PINKY.restrict_joint_angles(self.PINKY)
        self.RING = RING.restrict_joint_angles(self.RING)
        self.MIDDLE = MIDDLE.restrict_joint_angles(self.MIDDLE)
        self.INDEX = INDEX.restrict_joint_angles(self.INDEX)
        self.THUMB = THUMB.restrict_joint_angles(self.THUMB)

    def __getitem__(self, key : HandConfigIndex):
        """
        Return the value of the given key, with one of the following options:

            pinky_config = config.PINKY
            pinky_config = config["PINKY"]
            pinky_config = config[ids.SUBSYSTEMS.PINKY]
            pinky_config = config[0]
            pinky_abd = config[ids.JOINTS.PINKY_ABD]
        """
        if isinstance(key, str):
            key = key.upper()
            return getattr(self, key)
        elif isinstance(key, int):
            return getattr(self, list(ids.SUBSYSTEMS)[key].name)
        elif isinstance(key, ids.SUBSYSTEMS):
            return getattr(self, key.name)
        elif isinstance(key, ids.JOINTS):
            subsystem, index = self.joint_map[key]
            return self[subsystem][index]
        else:
            raise TypeError(f"key must be of type str, int, ids.SUBSYSTEMS or ids.JOINTS, not {type(key)}")


    def __setitem__(self, key : HandConfigIndex, value):
        """
        Set the value of the given key, with one of the following options:

            For all joints

            config.PINKY = [0, 0, 0]
            config["PINKY"] = [0, 0, 0]
            config[ids.SUBSYSTEMS.PINKY] = [0, 0, 0]
            config[0] = [0, 0, 0]

            For a single joint

            config[ids.JOINTS.PINKY_ABD] = 0

            NOTE: Assigning a value to a joint by doing configs[ids.SUBSYSTEMS.PINKY][0] = 0
                  Will NOT enforce the "triangle" restriction because __setitem__ is not called
                  I don't know how to fix this without creating an Observable List class
                  and overengineering, so please don't do that. Use a JOINT id instead to
                  modify a specific joint.
        """

        if type(value) is np.ndarray:
            value = value.tolist()

        if type(value) is not list:
            value = [float(value)]

        if isinstance(key, str):
            key = key.upper()
            setattr(self, key, value)
        elif isinstance(key, int):
            key = list(ids.SUBSYSTEMS)[key].name
            setattr(self, key, value)
        elif isinstance(key, ids.SUBSYSTEMS):
            key = key.name
            setattr(self, key, value)
        elif isinstance(key, ids.JOINTS):
            subsystem, index = self.joint_map[key]
            self[subsystem][index] = value
        else:
            raise TypeError(f"key must be of type str, int, ids.SUBSYSTEMS, or ids.JOINTS, not {type(key)}")

        self.restrict()

    def __iter__(self):
        """
        Iterate over the values of the HandConfig object
        """
        return iter([self.__getitem__(i) for i in range(len(ids.SUBSYSTEMS))])

    @property
    def ORIENTATION_XYZ(self):
        """
        Read-only property to get the orientation in euler angles
        """
        return R.from_quat(self.ORIENTATION).as_euler("xyz")

    def rotate_by_euler(self, euler):
        """
        Rotate quaternion by euler angles
        """
        rot = R.from_euler("xyz", euler)
        self.ORIENTATION = (rot * R.from_quat(self.ORIENTATION)).as_quat().tolist()


    @staticmethod
    def default():
        """
        Returns a configuration with all zeros
        """
        return HandConfig()

    @staticmethod
    def from_matrix(matrix, unrestricted = False):
        """
        Generates a hand configuration from a float matrix
        Only the n first elements of each row are used, where n is the number of joints in the subsystem
        """
        return HandConfig(
            unrestricted = unrestricted,
            PINKY = matrix[ids.SUBSYSTEMS.PINKY.value][:3],
            RING = matrix[ids.SUBSYSTEMS.RING.value][:3],
            MIDDLE = matrix[ids.SUBSYSTEMS.MIDDLE.value][:3],
            INDEX = matrix[ids.SUBSYSTEMS.INDEX.value][:3],
            THUMB = matrix[ids.SUBSYSTEMS.THUMB.value][:4],
            WRIST = matrix[ids.SUBSYSTEMS.WRIST.value][:1],
            POSITION = matrix[ids.SUBSYSTEMS.POSITION.value][:3],
            ORIENTATION = matrix[ids.SUBSYSTEMS.ORIENTATION.value][:4]
        )

    @staticmethod
    def read_current():
        """
        Generates a hand configuration from the current joint values
        """
        from dextrous_hand.subsystems.Finger import PINKY, RING, MIDDLE, INDEX, THUMB
        from dextrous_hand.subsystems.Wrist import WRIST
        from dextrous_hand.subsystems.Arm import ARM
        return HandConfig(PINKY = PINKY.read()[:3],
                          RING = RING.read()[:3],
                          MIDDLE = MIDDLE.read()[:3],
                          INDEX = INDEX.read()[:3],
                          THUMB = THUMB.read()[:4],
                          WRIST = WRIST.read()[:1],
                          POSITION = ARM.POSITION[:3],
                          ORIENTATION = ARM.ORIENTATION[:4]
                          )

    @staticmethod
    def from_msg(msg : Float32MultiArray, unrestricted = False):
        """
        Generates a hand configuration from a ROS Float32MultiArray message
        """
        config = HandConfig.from_matrix(utils.message_to_matrix(msg, len(ids.SUBSYSTEMS)), unrestricted)

        if not unrestricted:
            config.restrict()

        return config

    def as_matrix(self):
        """
        Returns the hand configuration as a matrix of floats
        """
        matrix = []
        self.restrict()
        for i in range(len(ids.SUBSYSTEMS)):
            matrix.append(self[i])
        return matrix

    def as_msg(self):
        """
        Returns the hand configuration as a ROS Float32MultiArray message
        """
        self.restrict()
        return utils.matrix_to_message(self.as_matrix())

    def pose_msg(self):
        """
        Returns the hand configuration as a ROS PoseStamped message
        """
        from dextrous_hand.subsystems.Arm import ARM
        return ARM.target_msg()

    def mujoco_angles(self):
        """
        Returns the hand configuration as a list of joint angles for Mujoco
        """
        print(f"Wrist: {self.WRIST}")
        arr = np.array(
                    self.WRIST + \
                    self.THUMB + \
                    self.INDEX + \
                    self.MIDDLE + \
                    self.RING + \
                    self.PINKY)

        # All joints except the wrist and the thumb's first two are rolling contact joints
        arr[3:] *= 0.5
        return arr

    def from_mujoco_angles(self, value):
        """
        Sets the hand configuration from a list of joint angles for Mujoco
        """
        array = np.array(value)
        array[3:] *= 2
        self.WRIST = [array[0]]
        self.THUMB = array[1:5].tolist()
        self.INDEX = array[5:8].tolist()
        self.MIDDLE = array[8:11].tolist()
        self.RING = array[11:14].tolist()
        self.PINKY = array[14:].tolist()

    @property
    def FINGERS(self):
        """
        Returns only the finger configurations
        """
        return [self[id] for id in ids.SUBSYSTEMS if id.value < ids.SUBSYSTEMS.WRIST.value]

    def __str__(self):
        """
        The string representation of the HandConfig object
        """
        d = self.__dict__.copy()
        d.pop("joint_map")
        d.pop("unrestricted")

        d["ORIENTATION_XYZ"] = self.ORIENTATION_XYZ

        return f"HandConfig(\n\t" + "\t".join([f"{key}=["+", ".join(f"{num:.3f}" for num in value)+"],\n" for key, value in d.items()]) + ")"

from dataclasses import dataclass
import dextrous_hand.utils as utils
from dextrous_hand import ids
from std_msgs.msg import Float32MultiArray

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
                    THUMB = [0, 0, 0],
                    WRIST = [0])
    """

    PINKY : list[float]
    RING : list[float]
    MIDDLE : list[float]
    INDEX : list[float]
    THUMB : list[float]
    WRIST : list[float]

    def __init__(self, **kwargs):
        """
        Initialize the HandConfig object with the given values or zeros if not provided

        params
            PINKY = [float, float, float]
            RING = [float, float, float]
            MIDDLE = [float, float, float]
            INDEX = [float, float, float]
            THUMB = [float, float, float]
            WRIST = [float]
        """
        self.PINKY = [0.0, 0.0, 0.0]
        self.RING = [0.0, 0.0, 0.0]
        self.MIDDLE = [0.0, 0.0, 0.0]
        self.INDEX = [0.0, 0.0, 0.0]
        self.THUMB = [0.0, 0.0, 0.0]
        self.WRIST = [0.0]

        from dextrous_hand.architecture import SUBSYSTEM_JOINTS
        self.joint_map = {}

        for subsystem in ids.SUBSYSTEMS:
            for joint in SUBSYSTEM_JOINTS[subsystem]:
                joint_ids = [joint.id for joint in SUBSYSTEM_JOINTS[subsystem]]
                index = joint_ids.index(joint.id)
                self.joint_map[joint.id] = (subsystem, index)

        for key, value in kwargs.items():
            if type(value) is float:
                value = [value]
            self[key] = value

        self.restrict()

    def restrict(self):
        from dextrous_hand.Finger import PINKY, RING, MIDDLE, INDEX, THUMB
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
            raise TypeError(f"key must be of type str, int, ids.SUBSYSTEMS, or Subsystem.Subsystem, not {type(key)}")


    def __setitem__(self, key : HandConfigIndex, value):
        """
        Set the value of the given key, with one of the following options:

            config.PINKY = [0, 0, 0]
            config["PINKY"] = [0, 0, 0]
            config[ids.SUBSYSTEMS.PINKY] = [0, 0, 0]
            config[0] = [0, 0, 0]
        """

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
            raise TypeError(f"key must be of type str, int, ids.SUBSYSTEMS, or Subsystem.Subsystem, not {type(key)}")
        self.restrict()

    def __iter__(self):
        """
        Iterate over the values of the HandConfig object
        """
        return iter([self.__getitem__(i) for i in range(len(ids.SUBSYSTEMS))])

    def __str__(self):
        """
        The string representation of the HandConfig object
        """
        d = self.__dict__.copy()
        d.pop("joint_map")

        return f"HandConfig(\n\t" + "\t".join([f"{key}=["+", ".join(f"{num:.3f}" for num in value)+"],\n" for key, value in d.items()]) + ")"

    @staticmethod
    def default():
        """
        Returns a configuration with all zeros
        """
        return HandConfig()

    @staticmethod
    def from_matrix(matrix):
        """
        Generates a hand configuration from a float matrix
        """
        return HandConfig(
            PINKY = matrix[ids.SUBSYSTEMS.PINKY.value][:3],
            RING = matrix[ids.SUBSYSTEMS.RING.value][:3],
            MIDDLE = matrix[ids.SUBSYSTEMS.MIDDLE.value][:3],
            INDEX = matrix[ids.SUBSYSTEMS.INDEX.value][:3],
            THUMB = matrix[ids.SUBSYSTEMS.THUMB.value][:3],
            WRIST = matrix[ids.SUBSYSTEMS.WRIST.value][:1],
        )

    @staticmethod
    def current():
        """
        Generates a hand configuration from the current joint values
        """
        from dextrous_hand.Finger import PINKY, RING, MIDDLE, INDEX, THUMB
        from dextrous_hand.Wrist import WRIST
        return HandConfig(PINKY = PINKY.read()[:3],
                          RING = RING.read()[:3],
                          MIDDLE = MIDDLE.read()[:3],
                          INDEX = INDEX.read()[:3],
                          THUMB = THUMB.read()[:3],
                          WRIST = WRIST.read()[:1])

    @staticmethod
    def from_msg(msg : Float32MultiArray):
        """
        Generates a hand configuration from a ROS Float32MultiArray message
        """
        config = HandConfig.from_matrix(utils.message_to_matrix(msg, len(ids.SUBSYSTEMS)))
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

    @property
    def FINGERS(self):
        """
        Returns only the finger configurations
        """
        return [self[finger] for finger in set(ids.SUBSYSTEMS).difference({ids.SUBSYSTEMS.WRIST})]

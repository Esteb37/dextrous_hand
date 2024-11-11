#!/usr/bin/env python3

from dextrous_hand.subsystems.Arm import ARM
import dextrous_hand.subsystems.Finger as Finger
from dextrous_hand.subsystems.Wrist import WRIST
from dextrous_hand.utils.HandConfig import HandConfig

class Hand():
    _instance = None

    def __new__(cls, *args, **kwargs):
        """
        Singleton pattern. Make sure only one instance of Hand is created.
        If it has already been created, return the existing instance
        """
        if cls._instance is None:
            cls._instance = super(Hand, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if hasattr(self, 'initialized') and self.initialized:
            return

        self.initialized = True

        self.target = HandConfig.default()

    def write_config(self, config : HandConfig):
        """
        Set all fingers, wrist and arm to the positions specified in the config

        params
            config: the configuration to set the hand to

        returns
            True if all subsystems are at position

        raises
            Exception: if the positions array has the wrong number of elements
        """
        self.target = config

        all_at_target = True
        for subsystem in Finger.FINGERS + [WRIST]:
            subsystem.write(config[subsystem.id])
            all_at_target = all_at_target and subsystem.at_target()

        ARM.write(config["POSITION"], config["ORIENTATION"])
        all_at_target = all_at_target and ARM.at_target()

        return all_at_target

    def read_current_config(self):
        """
        Read the current configuration of the hand and arm
        """
        return HandConfig.read_current()

    def get_target(self):
        """
        Get the target configuration of the hand and arm
        """
        return self.target

    def __str__(self):
        string = "--------Hand--------\n\nTarget:\n"
        string += str(self.get_target()) + "\n\nCurrent:\n"
        string += str(self.read_current_config()) + "\n"
        for subsystem in Finger.FINGERS + [WRIST]:
            string += str(subsystem) + "\n"
            for joint in subsystem.joints:
                string += "\t" + str(joint) + "\n"
                for motor in joint.motors:
                    string += "\t\t" + str(motor) + "\n"

        string += str(ARM) + "\n"
        return string

# Singleton instance
HAND = Hand()
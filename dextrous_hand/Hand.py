#!/usr/bin/env python3

from dextrous_hand.Finger import FINGERS
from dextrous_hand.Wrist import WRIST
from dextrous_hand.HandConfig import HandConfig

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

    def set_config(self, config : HandConfig):
        """
        Set all fingers and wrist to the positions specified in the config

        params
            config: the configuration to set the hand to

        returns
            True if all subsystems are at position

        raises
            Exception: if the positions array has the wrong number of elements
        """
        for subsystem in FINGERS + [WRIST]:
            subsystem.write(config[subsystem.id])
            if not subsystem.at_position():
                return False

        return True

    def get_config(self):
        return HandConfig.current()

    def __str__(self):
        string = ""
        for subsystem in FINGERS + [WRIST]:
            string += str(subsystem) + "\n"
            for joint in subsystem.joints:
                string += "\t" + str(joint) + "\n"
                for motor in joint.motors:
                    string += "\t\t" + str(motor) + "\n"
        return string

# Singleton instance
HAND = Hand()